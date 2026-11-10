// jpegcheck.c - does a baseline JPEG's compressed data end exactly where its
// last block ends? See jpegcheck.h. Plain C, no ESP-IDF deps (host-testable).
#include "jpegcheck.h"
#include <string.h>

typedef struct {
  bool    ok;
  int     mincode[17], maxcode[17], valptr[17];   // canonical Huffman (JPEG spec F.2.2.3)
  uint8_t val[256];
} huff_t;

typedef struct {
  const uint8_t *p, *end;
  uint32_t buf;          // next bits, MSB first; unused low bits are zero
  int      cnt;          // valid bits in buf
  bool     marker;       // reached a marker (or the end): no more data
} bits_t;

static void buildHuff(huff_t* h, const uint8_t* counts, const uint8_t* vals, int total) {
  memcpy(h->val, vals, total);
  int code = 0, k = 0;
  for (int l = 1; l <= 16; l++) {
    int n = counts[l - 1];
    h->valptr[l] = k;
    h->mincode[l] = code;
    h->maxcode[l] = n ? code + n - 1 : -1;
    code = (code + n) << 1;
    k += n;
  }
  h->ok = true;
}

static void fill(bits_t* b) {
  while (b->cnt <= 24 && !b->marker) {
    if (b->p >= b->end) { b->marker = true; break; }
    uint8_t c = b->p[0];
    if (c == 0xFF) {
      if (b->p + 1 < b->end && b->p[1] == 0x00) b->p += 2;   // stuffed FF
      else { b->marker = true; break; }                       // marker: stop before it
    } else {
      b->p++;
    }
    b->buf |= (uint32_t)c << (24 - b->cnt);
    b->cnt += 8;
  }
}

static int getBits(bits_t* b, int n) {
  if (n == 0) return 0;
  if (b->cnt < n) fill(b);
  if (b->cnt < n) return -1;                                  // data ran out
  int v = b->buf >> (32 - n);
  b->buf <<= n;
  b->cnt -= n;
  return v;
}

// ponytail: tries code lengths 1..16 in turn, ~44 ms per 640x360 image on the S3;
// add a 9-bit lookup table if the display task ever runs short of time
static int decode(bits_t* b, const huff_t* h) {
  if (b->cnt < 16) fill(b);
  uint32_t look = b->buf >> 16;
  for (int l = 1; l <= 16; l++) {
    int code = look >> (16 - l);
    if (code <= h->maxcode[l]) {
      if (l > b->cnt) return -1;                              // data ran out
      b->buf <<= l;
      b->cnt -= l;
      return h->val[(h->valptr[l] + code - h->mincode[l]) & 0xFF];
    }
  }
  return -1;                                                  // no such code
}

bool jpeg_complete(const uint8_t* j, size_t len) {
  if (len < 4 || j[0] != 0xFF || j[1] != 0xD8) return false;
  static huff_t dc[4], ac[4];                                 // single caller (display task)
  for (int t = 0; t < 4; t++) dc[t].ok = ac[t].ok = false;
  struct { int id, hs, vs, td, ta; } comp[4];
  int ncomp = 0, w = 0, h = 0, hmax = 1, vmax = 1;

  for (size_t i = 2;;) {
    if (i + 4 > len || j[i] != 0xFF) return false;
    uint8_t m = j[i + 1];
    if (m == 0xFF) { i++; continue; }                         // fill byte
    size_t L = j[i + 2] << 8 | j[i + 3];
    if (L < 2 || i + 2 + L > len) return false;
    const uint8_t* s = j + i + 4;
    size_t n = L - 2;

    if (m == 0xC4) {                                          // DHT: one or more tables
      while (n >= 17) {
        int total = 0;
        for (int l = 0; l < 16; l++) total += s[1 + l];
        if (total > 256 || 17 + (size_t)total > n) return false;
        buildHuff((s[0] >> 4) ? &ac[s[0] & 3] : &dc[s[0] & 3], s + 1, s + 17, total);
        s += 17 + total;
        n -= 17 + total;
      }
    } else if (m == 0xC0 || m == 0xC1) {                      // SOF0/1: baseline / extended Huffman
      if (n < 6) return false;
      h = s[1] << 8 | s[2];
      w = s[3] << 8 | s[4];
      ncomp = s[5];
      if (ncomp < 1 || ncomp > 4 || n < 6 + 3 * (size_t)ncomp || !w || !h) return false;
      for (int c = 0; c < ncomp; c++) {
        comp[c].id = s[6 + 3 * c];
        comp[c].hs = ncomp == 1 ? 1 : s[7 + 3 * c] >> 4;     // one component: 1 block per MCU
        comp[c].vs = ncomp == 1 ? 1 : s[7 + 3 * c] & 15;
        if (!comp[c].hs || !comp[c].vs) return false;
        if (comp[c].hs > hmax) hmax = comp[c].hs;
        if (comp[c].vs > vmax) vmax = comp[c].vs;
      }
    } else if (m >= 0xC2 && m <= 0xCF && m != 0xC4 && m != 0xC8 && m != 0xCC) {
      return true;                  // progressive / lossless / arithmetic: can't check, allow
    } else if (m == 0xDD) {
      if (n >= 2 && (s[0] | s[1])) return true;               // restart markers: not handled, allow
    } else if (m == 0xDA) {                                   // SOS: entropy data follows
      if (!ncomp || n < 1 || s[0] != ncomp) return true;      // multi-scan: not handled, allow
      if (n < 1 + 2 * (size_t)ncomp + 3) return false;
      for (int k = 0; k < ncomp; k++) {
        int c = 0;
        while (c < ncomp && comp[c].id != s[1 + 2 * k]) c++;
        if (c == ncomp) return false;
        comp[c].td = s[2 + 2 * k] >> 4 & 3;
        comp[c].ta = s[2 + 2 * k] & 3;
        if (!dc[comp[c].td].ok || !ac[comp[c].ta].ok) return false;
      }

      bits_t b = { j + i + 2 + L, j + len, 0, 0, false };
      int mcus = ((w + 8 * hmax - 1) / (8 * hmax)) * ((h + 8 * vmax - 1) / (8 * vmax));
      for (int mcu = 0; mcu < mcus; mcu++)
        for (int c = 0; c < ncomp; c++)
          for (int blk = comp[c].hs * comp[c].vs; blk; blk--) {
            int sz = decode(&b, &dc[comp[c].td]);             // DC: size category + bits
            if (sz < 0 || sz > 11 || getBits(&b, sz) < 0) return false;
            for (int k = 1; k < 64; k++) {                    // AC: run/size pairs
              int rs = decode(&b, &ac[comp[c].ta]);
              if (rs < 0) return false;
              int run = rs >> 4;
              sz = rs & 15;
              if (!sz) {
                if (run != 15) break;                         // EOB
                k += 15;                                      // ZRL: 16 zeros
                continue;
              }
              k += run;                                       // past 63 ends the block (libjpeg
              if (getBits(&b, sz) < 0) return false;          // tolerates it; Tapo emits it)
            }
          }

      // All blocks decoded: only padding bits may remain, then EOI.
      fill(&b);
      if (b.cnt >= 8) return false;                           // whole bytes left over
      const uint8_t* p = b.p;
      while (p + 2 < j + len && p[0] == 0xFF && p[1] == 0xFF) p++;
      return p + 1 < j + len && p[0] == 0xFF && p[1] == 0xD9;
    } else if (m == 0xD9) {
      return false;                                           // EOI before any scan
    }
    i += 2 + L;
  }
}
