/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: correction_link.h
 * Brief: Network thread fetching RTCM corrections from the LAN base or a caster
 *
 * Description:
 * 11 S3.2A RTG-1 (2026-11-02 ruling): differential input lives inside rtk_driver,
 * isolated in this thread so the network can only ever cost the corrections:
 *   (1) every socket call and DNS lookup runs here or in a detached helper,
 *       never in the 20 Hz loop;
 *   (2) the loop meets this thread at two mutex-guarded exchange points that
 *       only copy or swap -- takeFrames() and setPosition() never wait on I/O;
 *   (3) nothing thrown in here leaves the thread: the loop body and the thread
 *       entry both catch everything, close the connections and back off;
 *   (4) if this thread stops doing anything at all, the loop still publishes
 *       fix / heading / clock status; the receiver just loses corrections.
 *
 * Inside: a LAN source (primary, bare TCP to the RPi bridge, 11 S3.2A.1) and an
 * NTRIP source (backup, cold standby), each with its own Rtcm3Framer and
 * SourceHealth; one SourceSelector picks which source's frames go out (RTG-4).
 * The NTRIP source reports the robot's own fresh fix as GGA and rebuilds the
 * virtual station when the robot moves away from it (RTG-7 / RTG-12).
 *
 * RTG-9: the caster password is used only to build the request in memory; no
 * log line, status string or exception message here contains it.
 */

#ifndef SENSOR__CORRECTION_LINK_H_
#define SENSOR__CORRECTION_LINK_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "sensor/gga_vrs.h"
#include "sensor/rtcm_sink.h"
#include "sensor/rtcm_source.h"

namespace sensor {

struct LanSourceConfig {
  bool enabled;
  std::string host;            // numeric IP of the RPi bridge (RTB-1)
  int port;
  double connect_timeout_s;
  double read_timeout_s;       // streaming and silent this long -> reconnect
  double backoff_min_s;
  double backoff_max_s;
};

struct NtripSourceConfig {
  bool enabled;
  std::string host;
  int port;
  std::string mountpoint;
  int version;                 // 1 or 2
  std::string user;
  std::string password;        // RTG-9: never logged
  double connect_timeout_s;    // also bounds the DNS lookup
  double read_timeout_s;       // reply header, then silence while streaming
  double backoff_min_s;
  double backoff_max_s;
  double backoff_auth_s;       // after 401 / SOURCETABLE
  double gga_period_s;
  double gga_max_pos_age_s;
  double vrs_rebase_km;
};

// Everything the thread needs (configs rtcm.*). No defaults (CLAUDE.md 3.1).
struct CorrectionConfig {
  LanSourceConfig lan;
  NtripSourceConfig ntrip;
  HealthConfig health;
  SelectorConfig selector;     // linger_s comes from rtcm.ntrip.linger_s
  std::size_t queue_max_bytes; // out queue cap, same value as the RtcmSink cap
};

class CorrectionLink {
 public:
  explicit CorrectionLink(CorrectionConfig cfg);
  ~CorrectionLink();
  CorrectionLink(const CorrectionLink&) = delete;
  CorrectionLink& operator=(const CorrectionLink&) = delete;

  // True when at least one source is enabled; otherwise start() is a no-op.
  bool enabled() const { return cfg_.lan.enabled || cfg_.ntrip.enabled; }

  void start();
  void stop();   // sets the flag and joins; the poll timeout bounds the wait

  // ---- main-thread exchange points (copy / swap only) ----
  void setPosition(const PositionSample& s);
  void takeFrames(std::vector<TaggedFrame>* out);   // appends, oldest first
  std::string statusLine() const;

 private:
  struct Source;     // one connection + its framer / health (defined in the .cc)
  struct DnsJob;

  void run();
  void loopOnce();
  void stepLan(double now);
  void stepNtrip(double now, const PositionSample& pos);
  void onReadable(Source& s, bool is_ntrip, double now, std::vector<Rtcm3Frame>* frames);
  void fail(Source& s, double now, double backoff_override, const char* why);
  void closeAll(double now);
  void publishStatus(double now);
  void pushOut(const std::vector<Rtcm3Frame>& frames, int source, double now);

  CorrectionConfig cfg_;
  std::atomic<bool> stop_{false};
  std::thread thread_;

  // Network-thread state.
  std::unique_ptr<Source> lan_;
  std::unique_ptr<Source> ntrip_;
  SourceSelector selector_;
  VrsAnchor anchor_;
  std::shared_ptr<DnsJob> dns_;
  double last_gga_ = -1.0;

  // Exchange point 1: frames out (network -> main).
  std::mutex out_mu_;
  std::deque<TaggedFrame> out_;
  std::size_t out_bytes_ = 0;
  uint64_t out_dropped_ = 0;

  // Exchange point 2: position in (main -> network).
  std::mutex pos_mu_;
  PositionSample pos_;

  // Status snapshot for the heartbeat log.
  mutable std::mutex status_mu_;
  std::string status_;
};

}  // namespace sensor

#endif  // SENSOR__CORRECTION_LINK_H_
