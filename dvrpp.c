/******************************************************************************\
* Distance vector routing protocol without reverse path poisoning.             *
\******************************************************************************/

#include <stdlib.h>
#include <stdio.h>

#include "routing-simulator.h"

// Message format to send between nodes.
typedef struct {
  cost_t dv[MAX_NODES];
} message_t;

// State format.
typedef struct {
  cost_t costs[MAX_NODES][MAX_NODES];
  node_t next_hops[MAX_NODES];
} state_t;

int bellman_ford(state_t *state, node_t current_node, node_t neighbor, cost_t last_cost) {

  int send_msg = 0;

  for(node_t node =  get_first_node(); node <= get_last_node(); node++) {
    if(node != current_node){
      cost_t link_cost = COST_INFINITY;
      node_t next_hop = -1;
      for(node_t hop = get_first_node(); hop <= get_last_node(); hop++){

        cost_t hop_cost = get_link_cost(hop);
        if(hop_cost < COST_INFINITY && hop != current_node) {
          cost_t cost = COST_ADD(hop_cost, state->costs[hop][node]);
          if( cost < link_cost){
            link_cost = cost;
            next_hop = hop;

          }
        }
      }
      if(node == neighbor){
        if(last_cost != link_cost || (next_hop != -1 && state->costs[current_node][state->next_hops[next_hop]] == COST_INFINITY)) {
          state->next_hops[node] = next_hop;
          state->costs[current_node][node] = link_cost;
          set_route(node, next_hop, link_cost);
          send_msg = 1;
        }
      } else {
        if(state->costs[current_node][node] != link_cost ||  (next_hop != -1 && state->costs[current_node][state->next_hops[next_hop]] == COST_INFINITY)) {
          state->next_hops[node] = next_hop;
          state->costs[current_node][node] = link_cost;
          set_route(node, next_hop, link_cost);
          send_msg = 1;
        }
      }
    }
  }
  return send_msg;
}

// Notify a node that a neighboring link has changed cost.
void notify_link_change(node_t neighbor, cost_t new_cost) {
  state_t *s;
  if (!(s = (state_t*)get_state())) {

    s = (state_t*)malloc(sizeof(state_t));
    for (node_t i = 0; i < MAX_NODES; i++) {
      for (node_t j = 0; j < MAX_NODES; j++) {
        s->costs[i][j] = COST_INFINITY;
        if (i == j) s->costs[i][j] = 0;
      }
      s->next_hops[i] = -1;
    }
    set_state(s);
  }

  cost_t last_cost = s->costs[get_current_node()][neighbor];
  s->costs[get_current_node()][neighbor] = new_cost;

  int send_msg = bellman_ford(s, get_current_node(), neighbor, last_cost);

  if(send_msg) {
    for(node_t node =  get_first_node(); node <= get_last_node(); node++) {
      if(get_link_cost(node) < COST_INFINITY && node != get_current_node()){
        message_t *m = (message_t*)malloc(sizeof(message_t));
        cost_t cost;
        for (node_t i = 0; i < MAX_NODES; i++){
          if(s->next_hops[i] == node){
            cost = COST_INFINITY;
          } else {
            cost = s->costs[get_current_node()][i];
          }
          m->dv[i] = cost;
        }
        send_message(node,m);
      }
    }
  }
}



// Receive a message sent by a neighboring node.
void notify_receive_message(node_t sender, void *message) {
  state_t *s = (state_t*)get_state();

  cost_t last_cost = s->costs[get_current_node()][sender];

  for (node_t i = 0; i < MAX_NODES; i++)
    s->costs[sender][i] = ((message_t*)message)->dv[i];

  int send_msg = bellman_ford(s, get_current_node(), sender, last_cost);

  if(send_msg) {
    for(node_t node =  get_first_node(); node <= get_last_node(); node++) {
      if(get_link_cost(node) < COST_INFINITY && node != get_current_node()){
        message_t *m = (message_t*)malloc(sizeof(message_t));
        cost_t cost;
        for (node_t i = 0; i < MAX_NODES; i++){
          if(s->next_hops[i] == node){
            cost = COST_INFINITY;
          } else {
            cost = s->costs[get_current_node()][i];
          }
          m->dv[i] = cost;
        }
        send_message(node,m);
      }
    }
  }
}
