/*
 * Copyright (C) 2011, 2012, 2013 Citrix Systems
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE PROJECT AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "ns_turn_maps.h"
#include "ns_turn_msg_defs.h"

#include "ns_turn_ioalib.h"
#include "ns_ioalib_impl.h"

#include "turn_ports.h"

////////// DATA ////////////////////////////////////////////

#define PORTS_SIZE (0xFFFF+1)
#define TPS_OUT_OF_RANGE ((u32bits)(-1))
#define TPS_TAKEN_SINGLE ((u32bits)(-2))
#define TPS_TAKEN_EVEN ((u32bits)(-3))
#define TPS_TAKEN_ODD ((u32bits)(-4))

struct _turnports {
  u32bits status[PORTS_SIZE];
  u32bits low;
  u32bits high;
  u16bits range_start;
  u16bits range_stop;
  u16bits ports[PORTS_SIZE];
  TURN_MUTEX_DECLARE(mutex)
};
typedef struct _turnports turnports;

/////////////// TURNPORTS statics //////////////////////////

static turnports* turnports_create(super_memory_t *sm, u16bits start, u16bits end, uint16_t remote_port);
static u16bits turnports_size(turnports* tp);

static int turnports_allocate(turnports* tp, uint16_t remote_port);
static int turnports_allocate_even(turnports* tp, int allocate_rtcp, u64bits *reservation_token);

static void turnports_release(turnports* tp, u16bits port);

static int turnports_is_allocated(turnports* tp, u16bits port);
static int turnports_is_available(turnports* tp, u16bits port);
static void turnports_dump(turnports *tp, uint16_t remote_port);

/////////////// UTILS //////////////////////////////////////

static int is_taken(u32bits status) {
	int ret = -1;
	switch (status) {
	case TPS_TAKEN_SINGLE :
	case TPS_TAKEN_EVEN :
	case TPS_TAKEN_ODD :
		ret = 1;
		break;
	default:
		ret = 0;
	};
	return ret;
}

static void turnports_randomize(turnports* tp) {
  if(tp) {
    unsigned int size=(unsigned int)(tp->high-tp->low);
    unsigned int i=0;
    unsigned int cycles=size*10;
    for(i=0;i<cycles;i++) {
      u16bits port1 = (u16bits)(tp->low + (u16bits)(((unsigned long)random())%((unsigned long)size)));
      u16bits port2 = (u16bits)(tp->low + (u16bits)(((unsigned long)random())%((unsigned long)size)));
      if(port1!=port2) {
    	  int pos1=tp->status[port1];
    	  int pos2=tp->status[port2];
    	  int tmp=(int)tp->status[port1];
    	  tp->status[port1]=tp->status[port2];
    	  tp->status[port2]=(u32bits)tmp;
    	  tmp=(int)tp->ports[pos1];
    	  tp->ports[pos1]=tp->ports[pos2];
    	  tp->ports[pos2]=(u16bits)tmp;
      }
    }
  }
}   

static void turnports_init(turnports* tp, u16bits start, u16bits end, uint16_t remote_port) {
(void)remote_port;

  tp->low=start;
  tp->high=((u32bits)end)+1;

  tp->range_start=start;
  tp->range_stop=end;
  
  int i=0;
  for(i=0;i<start;i++) {
    tp->status[i]=TPS_OUT_OF_RANGE;
    tp->ports[i]=(u16bits)i;
  }
  for(i=start;i<=end;i++) {
    tp->status[i]=(u32bits)i;
    tp->ports[i]=(u16bits)i;
  }
  for(i=((int)end)+1;i<PORTS_SIZE;i++) {
    tp->status[i]=TPS_OUT_OF_RANGE;
    tp->ports[i]=(u16bits)i;
  }

  turnports_randomize(tp);
  /* turnports_dump(tp, remote_port); */

  TURN_MUTEX_INIT_RECURSIVE(&(tp->mutex));
}

/////////////// FUNC ///////////////////////////////////////

static volatile int dump_turnports = 0;

static void sigusr1_callback_handler(int signum) {
    (void)signum;
    dump_turnports = 1;
}

turnports* turnports_create(super_memory_t *sm, u16bits start, u16bits end, uint16_t remote_port) {

  if(start>end) return NULL;

    signal(SIGUSR1, sigusr1_callback_handler);

  turnports* ret=(turnports*)allocate_super_memory_region(sm, sizeof(turnports));
  turnports_init(ret,start,end, remote_port);

  return ret;
}

u16bits turnports_size(turnports* tp) {
  if(!tp) return 0;
  else {
    TURN_MUTEX_LOCK(&tp->mutex);
    u16bits ret = (u16bits)((tp->high-tp->low));
    TURN_MUTEX_UNLOCK(&tp->mutex);
    return ret;
  }
}

int turnports_allocate(turnports* tp, uint16_t remote_port) {

  int port=-1;

  TURN_MUTEX_LOCK(&tp->mutex);
  inc_file_counter("/var/log/turn_n_allocate");

  if (dump_turnports) {
    turnports_dump(tp, remote_port);
    dump_turnports = 0;
  }

  if(tp) {

    while(1) {
      
      if(tp->high <= tp->low) {
    	  TURN_MUTEX_UNLOCK(&tp->mutex);
          dlog("%hu: turnports_allocate: tp->low: %u, tp->high: %u", remote_port, tp->low, tp->high);
    	  return -1;
      }
      
      int position=(u16bits)(tp->low & 0x0000FFFF);
      
      port=(int)tp->ports[position];
      if(port<(int)(tp->range_start) || port>((int)(tp->range_stop))) {
    	  TURN_MUTEX_UNLOCK(&tp->mutex);
          dlog("%hu: turnports_allocate: port: %i, tp->range_start: %hu, tp->range_stop: %hu", remote_port, port, tp->range_start, tp->range_stop);
    	  return -1;
      }
      if(is_taken(tp->status[port])) {
    	  ++(tp->low);
    	  continue;
      } 
      if(tp->status[port]!=tp->low) {
    	  ++(tp->low);
    	  continue;
      }
      tp->status[port]=TPS_TAKEN_SINGLE;
      ++(tp->low);
      break;
    }
  }

  TURN_MUTEX_UNLOCK(&tp->mutex);

  return port;
}

void turnports_release(turnports* tp, u16bits port) {
  TURN_MUTEX_LOCK(&tp->mutex);
  if(tp && port>=tp->range_start && port<=tp->range_stop) {
    u16bits position=(u16bits)(tp->high & 0x0000FFFF);
    if(is_taken(tp->status[port])) {
      tp->status[port]=tp->high;
      tp->ports[position]=port;
      ++(tp->high);
    }
  }
  TURN_MUTEX_UNLOCK(&tp->mutex);
}

int turnports_allocate_even(turnports* tp, int allocate_rtcp, u64bits *reservation_token) {
  if(tp) {
    TURN_MUTEX_LOCK(&tp->mutex);
    u16bits size = turnports_size(tp);
    if(size>1) {
      u16bits i=0;
      for(i=0;i<size;i++) {
    	  int port=turnports_allocate(tp, 0);
    	  if(port & 0x00000001) {
    		  turnports_release(tp,port);
    	  } else {
    		  if(!allocate_rtcp) {
    			  TURN_MUTEX_UNLOCK(&tp->mutex);
    			  return port;
    		  } else {
    			  int rtcp_port=port+1;
    			  if(rtcp_port>tp->range_stop) {
    				  turnports_release(tp,port);
    			  } else if(!turnports_is_available(tp,rtcp_port)) {
    				  turnports_release(tp,port);
    			  } else {
    				  tp->status[port]=TPS_TAKEN_EVEN;
    				  tp->status[rtcp_port]=TPS_TAKEN_ODD;
    				  if(reservation_token) {
    					  u16bits *v16=(u16bits*)reservation_token;
    					  u32bits *v32=(u32bits*)reservation_token;
    					  v16[0]=(u16bits)(tp->ports[(u16bits)(tp->low & 0x0000FFFF)]);
    					  v16[1]=(u16bits)(tp->ports[(u16bits)(tp->high & 0x0000FFFF)]);
    					  v32[1]=(u32bits)turn_random();
    				  }
    				  TURN_MUTEX_UNLOCK(&tp->mutex);
    				  return port;
    			  }
    		  }
    	  }
      }
    }
    TURN_MUTEX_UNLOCK(&tp->mutex);
  }
  return -1;
}

int turnports_is_allocated(turnports* tp, u16bits port) {
  if(!tp) return 0;
  else {
    TURN_MUTEX_LOCK(&tp->mutex);
    int ret = is_taken(tp->status[port]);
    TURN_MUTEX_UNLOCK(&tp->mutex);
    return ret;
  }
}

int turnports_is_available(turnports* tp, u16bits port) {
  if(tp) {
    TURN_MUTEX_LOCK(&tp->mutex);
    u32bits status = tp->status[port];
    if((status!=TPS_OUT_OF_RANGE) && !is_taken(status)) {
      u16bits position=(u16bits)(status & 0x0000FFFF);
      if(tp->ports[position]==port) {
    	  TURN_MUTEX_UNLOCK(&tp->mutex);
    	  return 1;
      }
    }
    TURN_MUTEX_UNLOCK(&tp->mutex);
  }
  return 0;
}

/*
#define STATUS_TO_STRING_MSG "<error>"
static char *status_to_string(u32bits status, char *buf, size_t sz) {
    strncpyex(buf, STATUS_TO_STRING_MSG, sz);
    switch (status) {
    case TPS_OUT_OF_RANGE:
        snprintf(buf, sz, "out of range");
        return buf;
    case TPS_TAKEN_SINGLE:
        snprintf(buf, sz, "taken, single");
        return buf;
    case TPS_TAKEN_EVEN:
        snprintf(buf, sz, "taken, even");
        return buf;
    case TPS_TAKEN_ODD:
        snprintf(buf, sz, "taken, odd");
        return buf;
    default:
        snprintf(buf, sz, "%u", status);
        return buf;
    }
}
*/

#define status_to_string(tp, port) \
    status_out_of_range(tp, port) ? "out of range" \
    : status_taken(tp, port) ? "taken" \
    : status_available(tp, port) ? "available" \
    : "invalid"
#define status_out_of_range(tp, port) \
    tp->status[port] == TPS_OUT_OF_RANGE
#define status_taken(tp, port) \
    (tp->status[port] == TPS_TAKEN_SINGLE \
        || tp->status[port] == TPS_TAKEN_EVEN \
        || tp->status[port] == TPS_TAKEN_ODD)
#define status_available(tp, port) \
    tp->ports[tp->status[port] & 0xFFFF] == port
#define status_invalid(tp, port) \
    (! status_out_of_range(tp, port) \
    && ! status_taken(tp, port) \
    && ! status_available(tp, port))

#define TURNPORTS_DUMP_BUFSIZE 2000000
static void turnports_dump(turnports *tp, uint16_t remote_port) {
    char s[TURNPORTS_DUMP_BUFSIZE], *p;
    int r, port, n_out_of_range, n_taken, n_available, n_invalid, n_port_out_of_range;
    u32bits i;

    p = s;
    n_out_of_range = n_taken = n_available = n_invalid = 0;
    for (port = tp->range_start; port <= tp->range_stop; port++) {
        if (status_out_of_range(tp, port)) n_out_of_range++;
        if (status_taken(tp, port)) n_taken++;
        if (status_available(tp, port)) n_available++;
        if (status_invalid(tp, port)) n_invalid++;
        /*
        r = snprintf(p, TURNPORTS_DUMP_BUFSIZE - (p - s), "%i: %s\n",
            port,
            status_to_string(tp, port));
        p += r;
        */
    }
    r = snprintf(p, TURNPORTS_DUMP_BUFSIZE - (p - s), "--- status scan\n");
    p += r;
    r = snprintf(p, TURNPORTS_DUMP_BUFSIZE - (p - s),
        "out of range: %i, taken: %i, available: %i, invalid: %i\n",
        n_out_of_range, n_taken, n_available, n_invalid);
    p += r;

    n_out_of_range = n_taken = n_available = n_invalid = n_port_out_of_range = 0;
    for (i = tp->low; i < tp->high; i++) {
        int port = tp->ports[i & 0xFFFF];
        if (status_out_of_range(tp, port)) n_out_of_range++;
        if (status_taken(tp, port)) n_taken++;
        if (status_available(tp, port)) n_available++;
        if (status_invalid(tp, port)) n_invalid++;
        if (port < tp->range_start || port > tp->range_stop) n_port_out_of_range++;
        if (
            status_out_of_range(tp, port)
            || status_taken(tp, port)
            || status_invalid(tp, port)
            || port < tp->range_start
            || port > tp->range_stop
        ) {
            r = snprintf(p, TURNPORTS_DUMP_BUFSIZE - (p - s), "%i: %s\n",
                port,
                port < tp->range_start
                || port > tp->range_stop
                    ? "port out of range"
                    : status_to_string(tp, port));
            p += r;
        }
    }
    r = snprintf(p, TURNPORTS_DUMP_BUFSIZE - (p - s), "--- port scan\n");
    p += r;
    r = snprintf(p, TURNPORTS_DUMP_BUFSIZE - (p - s),
        "out of range: %i, taken: %i, available: %i, invalid: %i, port out of range: %i",
        n_out_of_range, n_taken, n_available, n_invalid, n_port_out_of_range);
    p += r;

    dlog("%hu: turnports: range_start: %hu, range_stop: %hu\nlow: %u, high: %u, size: %hu\n%s",
        remote_port, tp->range_start, tp->range_stop,
        tp->low, tp->high, turnports_size(tp),
        s);
}

/////////////////// IP-mapped PORTS /////////////////////////////////////

struct _turnipports
{
	super_memory_t *sm;
	u16bits start;
	u16bits end;
	ur_addr_map ip_to_turnports_udp;
	ur_addr_map ip_to_turnports_tcp;
	TURN_MUTEX_DECLARE(mutex)
};

//////////////////////////////////////////////////

static ur_addr_map *get_map(turnipports *tp, u08bits transport)
{
	if(transport == STUN_ATTRIBUTE_TRANSPORT_TCP_VALUE)
		return &(tp->ip_to_turnports_tcp);
	return &(tp->ip_to_turnports_udp);
}
//////////////////////////////////////////////////

static turnipports* turnipports_singleton = NULL;

turnipports* turnipports_create(super_memory_t *sm, u16bits start, u16bits end)
{
	turnipports *ret = (turnipports*) allocate_super_memory_region(sm, sizeof(turnipports));
	ret->sm = sm;
	ur_addr_map_init(&(ret->ip_to_turnports_udp));
	ur_addr_map_init(&(ret->ip_to_turnports_tcp));
	ret->start = start;
	ret->end = end;
	TURN_MUTEX_INIT_RECURSIVE(&(ret->mutex));
	turnipports_singleton = ret;
	return ret;
}

static turnports* turnipports_add(turnipports* tp, u08bits transport, const ioa_addr *backend_addr, uint16_t remote_port)
{
	ur_addr_map_value_type t = 0;
	if (tp && backend_addr) {
		ioa_addr ba;
		addr_cpy(&ba, backend_addr);
		addr_set_port(&ba, 0);
        char _s[60];
		TURN_MUTEX_LOCK((const turn_mutex*)&(tp->mutex));
		if (!ur_addr_map_get(get_map(tp, transport), &ba, &t)) {
			t = (ur_addr_map_value_type) turnports_create(tp->sm, tp->start, tp->end, remote_port);
            turnports *t2 = (turnports *)t;
            dlog("%hu: created turnports: %s: %hu, %hu, %u, %u",
                remote_port,
                ioa_addr_to_string(&ba, _s),
                t2->range_start, t2->range_stop, t2->low, t2->high);
			ur_addr_map_put(get_map(tp, transport), &ba, t);
		} else {
            turnports *t2 = (turnports *)t;
            dlog("%hu: got turnports: %s: %hu, %hu, %u, %u",
                remote_port,
                ioa_addr_to_string(&ba, _s),
                t2->range_start, t2->range_stop, t2->low, t2->high);
        }
		TURN_MUTEX_UNLOCK((const turn_mutex*)&(tp->mutex));
	}
	return (turnports*) t;
}

void turnipports_add_ip(u08bits transport, const ioa_addr *backend_addr)
{
	turnipports_add(turnipports_singleton, transport, backend_addr, 0);
}

int turnipports_allocate(turnipports* tp, u08bits transport, const ioa_addr *backend_addr, uint16_t remote_port)
{
	int ret = -1;
	if (tp && backend_addr) {
		TURN_MUTEX_LOCK((const turn_mutex*)&(tp->mutex));
		turnports *t = turnipports_add(tp, transport, backend_addr, remote_port);
		ret = turnports_allocate(t, remote_port);
        dlog("%hu: turnports_allocate: %i", remote_port, ret);
		TURN_MUTEX_UNLOCK((const turn_mutex*)&(tp->mutex));
	}
	return ret;
}

int turnipports_allocate_even(turnipports* tp, const ioa_addr *backend_addr, int allocate_rtcp,
				u64bits *reservation_token)
{
	int ret = -1;
	if (tp && backend_addr) {
		TURN_MUTEX_LOCK((const turn_mutex*)&(tp->mutex));
		turnports *t = turnipports_add(tp, STUN_ATTRIBUTE_TRANSPORT_UDP_VALUE, backend_addr, 0);
		ret = turnports_allocate_even(t, allocate_rtcp, reservation_token);
		TURN_MUTEX_UNLOCK((const turn_mutex*)&(tp->mutex));
	}
	return ret;
}

void turnipports_release(turnipports* tp, u08bits transport, const ioa_addr *socket_addr)
{
	if (tp && socket_addr) {
		ioa_addr ba;
		ur_addr_map_value_type t;
		addr_cpy(&ba, socket_addr);
		addr_set_port(&ba, 0);
		TURN_MUTEX_LOCK((const turn_mutex*)&(tp->mutex));
		if (ur_addr_map_get(get_map(tp, transport), &ba, &t)) {
			turnports_release((turnports*) t, addr_get_port(socket_addr));
		}
		TURN_MUTEX_UNLOCK((const turn_mutex*)&(tp->mutex));
	}
}

int turnipports_is_allocated(turnipports* tp, u08bits transport, const ioa_addr *backend_addr, u16bits port)
{
	int ret = 0;
	if (tp && backend_addr) {
		ioa_addr ba;
		ur_addr_map_value_type t;
		addr_cpy(&ba, backend_addr);
		addr_set_port(&ba, 0);
		TURN_MUTEX_LOCK((const turn_mutex*)&(tp->mutex));
		if (ur_addr_map_get(get_map(tp,transport), &ba, &t)) {
			ret = turnports_is_allocated((turnports*) t, port);
		}
		TURN_MUTEX_UNLOCK((const turn_mutex*)&(tp->mutex));
	}
	return ret;
}

int turnipports_is_available(turnipports* tp, u08bits transport, const ioa_addr *backend_addr, u16bits port)
{
	int ret = 0;
	if (tp && backend_addr) {
		ioa_addr ba;
		ur_addr_map_value_type t;
		addr_cpy(&ba, backend_addr);
		addr_set_port(&ba, 0);
		TURN_MUTEX_LOCK((const turn_mutex*)&(tp->mutex));
		if (!ur_addr_map_get(get_map(tp,transport), &ba, &t)) {
			ret = 1;
		} else {
			ret = turnports_is_available((turnports*) t, port);
		}
		TURN_MUTEX_UNLOCK((const turn_mutex*)&(tp->mutex));
	}
	return ret;
}

//////////////////////////////////////////////////////////////////


