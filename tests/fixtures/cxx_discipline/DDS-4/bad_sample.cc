// bad: creating a writer on domain 0 writes into the vendor domain.
extern "C" int dds_create_writer(int p, int t, const void* q, void* l);
void wire(int p, int t) { dds_create_writer(p, t, nullptr, nullptr); }
