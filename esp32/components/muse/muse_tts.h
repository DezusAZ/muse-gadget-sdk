// Spoken replies for Muse gadgets: turns a reply's text into MP3 audio via a
// text-to-speech service, so the decode path in muse_chat_session can play it.
//
// The stock firmware doesn't speak gadget replies (start_tts() just paces
// silence). This module fills that hook: muse_tts_request(text) fetches the
// speech as MP3 on a background task and streams the bytes into an internal
// buffer; the session task pulls them with muse_tts_read() and feeds the
// existing minimp3 decoder. The provider lives behind build_tts_url() in the
// .c, so swapping Google TTS for OpenAI, a self-hosted Piper, etc. is a local
// change, not a rewrite.
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Create the MP3 handoff buffer and the fetch task. Safe to call more than
// once; only the first call does anything. No network happens until a request.
void muse_tts_init(void);

// True once muse_tts_init() has set up the buffer and task.
bool muse_tts_ready(void);

// Start fetching speech for `text` (copied immediately). Aborts any fetch
// already in flight and clears buffered audio first. Returns right away; the
// MP3 arrives on the background task.
void muse_tts_request(const char *text);

// Abort the current fetch and discard any buffered audio.
void muse_tts_abort(void);

// True while a fetch is in progress. May briefly be false while the last bytes
// still sit in the buffer, so check muse_tts_available() too before concluding
// the stream has ended.
bool muse_tts_busy(void);

// Bytes of MP3 currently buffered and ready to read.
size_t muse_tts_available(void);

// Copy up to `max` buffered MP3 bytes into `dst`. Non-blocking; returns the
// number of bytes copied (0 if none ready).
size_t muse_tts_read(uint8_t *dst, size_t max);

#ifdef __cplusplus
}
#endif
