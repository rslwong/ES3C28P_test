#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// True if this baseline JPEG's compressed data holds exactly the blocks its
// header promises - no more, no less - and then ends with EOI. A "torn" image
// (head of one JPEG + tail of another) practically never does: libjpeg would
// call it "premature end of data segment" or "extraneous bytes before marker".
// Walks the Huffman codes only (no IDCT). Host test: make test.
// Formats it doesn't handle (progressive, restart markers, multi-scan) -> true.
bool jpeg_complete(const uint8_t* jpg, size_t len);
