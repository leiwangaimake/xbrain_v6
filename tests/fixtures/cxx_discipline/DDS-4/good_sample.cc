// good: readers only on domain 0.
extern "C" int dds_create_reader(int p, int t, const void* q, void* l);
void wire(int p, int t) { dds_create_reader(p, t, nullptr, nullptr); }
