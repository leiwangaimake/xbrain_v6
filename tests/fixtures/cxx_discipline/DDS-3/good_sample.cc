// good: config injected in-process, per domain.
extern "C" int dds_create_domain(unsigned id, const char* cfg);
void setup() { dds_create_domain(0, "<CycloneDDS><Domain/></CycloneDDS>"); }
