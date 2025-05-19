/*
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#define BTSTACK_FILE__ "btstack_tlv_ac695n.c"

#include "btstack_tlv.h"
#include "btstack_util.h"
#include "btstack_debug.h"

#include <inttypes.h>
#include <string.h>

static nvs_handle the_nvs_handle;
static int nvs_active;

// @param buffer char array of size 9
static void key_for_tag(uint32_t tag, char * key_buffer){
	int i;
	for (i=0;i<8;i++){
		key_buffer[i] = char_for_nibble( (tag >> (4*(7-i))) & 0x0f);
	}
	key_buffer[i] = 0;
}

/**
 * Get Value for Tag
 * @param tag
 * @param buffer
 * @param buffer_size
 * @returns size of value
 */
static int btstack_tlv_ac695n_get_tag(void * context, uint32_t tag, uint8_t * buffer, uint32_t buffer_size){
	if (!nvs_active) return 0;
	char key_buffer[9];
	key_for_tag(tag, key_buffer);
	log_debug("read tag %s", key_buffer);
}

/**
 * Store Tag 
 * @param tag
 * @param data
 * @param data_size
 */
static int btstack_tlv_ac695n_store_tag(void * context, uint32_t tag, const uint8_t * data, uint32_t data_size){
	if (!nvs_active) return 0;
	char key_buffer[9];
	key_for_tag(tag, key_buffer);
	log_info("store tag %s", key_buffer);
}

/**
 * Delete Tag
 * @param tag
 */
static void btstack_tlv_ac695n_delete_tag(void * context, uint32_t tag){
	if (!nvs_active) return;
	char key_buffer[9];
	key_for_tag(tag, key_buffer);
	log_info("delete tag %s", key_buffer);
}

static const btstack_tlv_t btstack_tlv_ac695n = {
	/* int  (*get_tag)(..);     */ &btstack_tlv_ac695n_get_tag,
	/* int (*store_tag)(..);    */ &btstack_tlv_ac695n_store_tag,
	/* void (*delete_tag)(v..); */ &btstack_tlv_ac695n_delete_tag,
};

/**
 * Init Tag Length Value Store
 */
const btstack_tlv_t * btstack_tlv_ac695n_get_instance(void){
	// Initialize NVS
	return &btstack_tlv_ac695n;
}

