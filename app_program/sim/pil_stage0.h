/* Stage 0 selection guard: no operational sim devices or link yet. */
#ifndef PIL_STAGE0_H
#define PIL_STAGE0_H
#if !defined(APP_SIM) || APP_SIM != 1
#error "PIL skeletons require SIM=1"
#endif
#endif
