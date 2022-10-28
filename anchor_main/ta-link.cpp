#include "ta-link.h"
#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugF(x, y) Serial.print(x, y)
#define debuglnF(x, y) Serial.println(x, y)
#else
#define debug(x)
#define debugln(x)
#define debugF(x, y)
#define debuglnF(x, y)  //13470
#endif

struct TALink* init_link() {
  struct TALink* p = (struct TALink*)malloc(sizeof(struct TALink));
  p->next = NULL;
  p->anchor_addr = 0;
  p->last_time = millis();
  p->range[0] = 0.0;
  p->range[1] = 0.0;
  p->range[2] = 0.0;

  return p;
}

void add_link(struct TALink* p, uint16_t addr) {
  struct TALink* temp = p;
  while (temp->next != NULL) {
    temp = temp->next;
  }

  struct TALink* a = (struct TALink*)malloc(sizeof(struct TALink));
  a->anchor_addr = addr;
  a->range[0] = 0.0;
  a->range[1] = 0.0;
  a->range[2] = 0.0;
  a->dbm = 0.0;
  p->last_time = millis();
  a->next = NULL;

  temp->next = a;
}

struct TALink* find_link(struct TALink* p, uint16_t addr) {
  if (addr == 0) {
    return NULL;
  }

  if (p->next == NULL) {
    return NULL;
  }

  struct TALink* temp = p;

  while (temp->next != NULL) {
    temp = temp->next;
    if (temp->anchor_addr == addr) {
      return temp;
    }
  }

  debugln("find_link:Can't find addr");
  return NULL;
}

void fresh_link(struct TALink* p, uint16_t addr, float range, float dbm, uint16_t threshold) {
  struct TALink* temp = find_link(p, addr);
  if (temp != NULL) {
    unsigned long cur = millis();
    if (cur - temp->last_time > threshold) {
      temp->range[2] = 0;
      temp->range[1] = 0;
      temp->range[0] = 0;
    }
    temp->last_time = cur;
    temp->range[2] = temp->range[1];
    temp->range[1] = temp->range[0];
    temp->range[0] = (range + temp->range[1] + temp->range[2]) / 3;
    temp->dbm = dbm;
    return;
  } else {
    debugln("fresh_link:Fresh fail");
    return;
  }
}

void print_link(struct TALink* p) {
  struct TALink* temp = p;

  while (temp->next != NULL) {
    //Serial.println("Dev %d:%d m", temp->next->anchor_addr, temp->next->range);
    debuglnF(temp->next->anchor_addr, HEX);
    debugln(temp->next->range[0]);
    debugln(temp->next->dbm);
    temp = temp->next;
  }

  return;
}

void delete_link(struct TALink* p, uint16_t addr) {
  if (addr == 0)
    return;

  struct TALink* temp = p;
  while (temp->next != NULL) {
    if (temp->next->anchor_addr == addr) {
      struct TALink* del = temp->next;
      temp->next = del->next;
      free(del);
      return;
    }
    temp = temp->next;
  }
  return;
}

void to_json(struct TALink* p, String* s) {
  struct TALink* temp = p;

  while (temp->next != NULL) {
    temp = temp->next;
    debug("Range: ");
    char link_json[50];
    debug("Anchor Addr: ");
    debug( temp->anchor_addr);
    debug("\t\t Distance: ");
    debugln(temp->range[0]);
  }
}

uint8_t getLen(struct TALink* p) {
  uint8_t count = 0;
  struct TALink* temp = p;
  while (temp->next != NULL) {
    count++;
    temp = temp->next;
  }
  return count;
}
