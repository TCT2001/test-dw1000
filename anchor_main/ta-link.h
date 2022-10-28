#include <Arduino.h>
#define RESULT_NUM 3

struct TALink {
  uint16_t anchor_addr;
  float range[RESULT_NUM];
  float dbm;
  unsigned long last_time;
  struct TALink *next;
};

struct TALink *init_link();
void add_link(struct TALink *p, uint16_t addr);
void delete_link(struct TALink *p, uint16_t addr);
struct TALink *find_link(struct TALink *p, uint16_t addr);
void fresh_link(struct TALink *p, uint16_t addr, float range, float dbm, uint16_t threshold);
void print_link(struct TALink *p);
void to_json(struct TALink *p, String *s);
uint8_t getLen(struct TALink *p);
