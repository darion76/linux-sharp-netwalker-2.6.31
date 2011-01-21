/*
 *   Driver for KeyStream wireless LAN
 *   
 *   michael_mic.h
 *   $Id: michael_mic.h 623 2008-06-20 08:09:31Z sekine $
 *
 *   Copyright (c) 2005-2008 KeyStream Corp.
 *   All rights reserved.
 *
 */

/* MichelMIC routine define */
struct michel_mic_t {
	uint32_t K0;	// Key 
	uint32_t K1;	// Key 
	uint32_t L;		// Current state 
	uint32_t R;		// Current state 
	uint8_t M[4];		// Message accumulator (single word) 
	int		nBytesInM; 	// # bytes in M 
	uint8_t 	Result[8];
};

extern
void MichaelMICFunction( struct michel_mic_t *Mic, uint8_t *Key, 
			 uint8_t *Data, int Len, uint8_t priority, 
			 uint8_t *Result );
