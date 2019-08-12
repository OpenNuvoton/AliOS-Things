/* Microcontroller Library
 * Copyright (c) 2015-2016 Nuvoton
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <string.h>

#include "nu_rbuf.h"

#define MIN(a, b) (a)<(b)? (a) : (b)

int nu_rbuf_create(nu_rbuf_t *nu_rbuf, char *buffer, int length )
{
    if ( (nu_rbuf == NULL) || (buffer == NULL)) {
        return -1;
    }
	
    memset(nu_rbuf, 0, sizeof(nu_rbuf_t));
    memset(buffer, 0, length);

    nu_rbuf->length = length;
    nu_rbuf->head = 0;
    nu_rbuf->tail = 0;
		nu_rbuf->chunk_idx = 0;
    nu_rbuf->buffer = (uint8_t *)buffer;

    return 0;
}

int nu_rbuf_dump(nu_rbuf_t *nu_rbuf )
{
    if ( (nu_rbuf == NULL)) {
        return -1;
    }	
		printf("[%08x, %08x-%d] r=%d, w=%d, ci=%d \n", nu_rbuf, nu_rbuf->buffer, nu_rbuf->length, nu_rbuf->head, nu_rbuf->tail, nu_rbuf->chunk_idx );
    return 0;
}

void nu_rbuf_destroy(nu_rbuf_t *nu_rbuf)
{
    memset(nu_rbuf->buffer, 0, nu_rbuf->length);
    nu_rbuf->length = 0;
    nu_rbuf->head = nu_rbuf->tail = 0;
}

int nu_rbuf_avail_read_space(nu_rbuf_t *nu_rbuf)
{
		int ret=0;

    if (nu_rbuf->head == nu_rbuf->tail) {
        ret = 0;
    } else if (nu_rbuf->head < nu_rbuf->tail) {
        ret = nu_rbuf->tail - nu_rbuf->head;
    } else {
        ret = nu_rbuf->length - (nu_rbuf->head - nu_rbuf->tail);
    }
		
		return ret;
}

int nu_rbuf_write(nu_rbuf_t *nu_rbuf, uint8_t *data, uint32_t length)
{
    int i = 0;

    if (nu_rbuf == NULL || data == NULL || length == 0) {
        return -1;
    }

    /* if empty reset head and tail to zero */
    if (nu_rbuf_empty(nu_rbuf)) {
        nu_rbuf_clear(nu_rbuf);
    }

    for (i = 0; i < length; i++) {

        if (nu_rbuf_full(nu_rbuf)) {
            //printf("0x%x ringbuffer %d full\r\n", nu_rbuf->buffer,nu_rbuf->length);
            break;
        }

        nu_rbuf->buffer[nu_rbuf->tail] = data[i];

        nu_rbuf->tail++;
        nu_rbuf->tail %= (nu_rbuf->length);

    }

    /* return real write len */
    return i;
}

int nu_rbuf_dummy_write(nu_rbuf_t * nu_rbuf, uint32_t length)
{
	 if (nu_rbuf == NULL || length == 0)
        return -1;
 
	 nu_rbuf->tail += length;
	 nu_rbuf->tail %= (nu_rbuf->length);
	 return length;
}

int nu_rbuf_read(nu_rbuf_t *nu_rbuf, uint8_t *target, uint32_t amount)
{
    int copy_sz = 0;
    int i;

    if (nu_rbuf == NULL || target == NULL || amount == 0) {
        return -1;
    }

    if (nu_rbuf_empty(nu_rbuf)) {
        goto exit_nu_rbuf_read;
    }

    /* get real read size */
    copy_sz = MIN(amount, nu_rbuf_avail_read_space(nu_rbuf));

    /* cp data to user buffer */
    for (i = 0; i < copy_sz; i++) {
        target[i] = nu_rbuf->buffer[nu_rbuf->head];

        nu_rbuf->head++;
        nu_rbuf->head %= (nu_rbuf->length);
    }

exit_nu_rbuf_read:
    return copy_sz;
}
