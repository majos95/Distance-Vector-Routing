/******************************************************************************\
* Path vector routing protocol.                                                *
\******************************************************************************/



#include <stdlib.h>
#include <stdio.h>

#include "routing-simulator.h"


// Message format to send between nodes.
typedef struct {
  cost_t dv[MAX_NODES];
  node_t routes[MAX_NODES][MAX_NODES];
} message_t;

// State format.
typedef struct {
  cost_t costs[MAX_NODES][MAX_NODES];
  node_t next_hops[MAX_NODES][MAX_NODES];
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
        if(last_cost != link_cost || (next_hop != -1 && state->costs[current_node][state->next_hops[current_node][next_hop]] == COST_INFINITY)) {

          if(link_cost > last_cost && state->next_hops[current_node][node] != next_hop){
            state->next_hops[current_node][node] = -1;
            for(node_t i = 0; i < MAX_NODES; i++){
              if (state->next_hops[i][current_node] == node) {
                state->next_hops[i][current_node] = -1;
                state->costs[current_node][i] = COST_INFINITY;
                state->costs[next_hop][i] = COST_INFINITY;
                state->costs[node][next_hop] = COST_INFINITY;
                set_route(i, node, COST_INFINITY);
              }
            }
            link_cost = COST_INFINITY;
          } else {
            state->next_hops[current_node][node] = next_hop;
          }
          state->costs[current_node][node] = link_cost;
          set_route(node, next_hop, link_cost);
          send_msg = 1;
        }
      } else {
        if(state->costs[current_node][node] != link_cost ||  (next_hop != -1 && state->costs[current_node][state->next_hops[current_node][next_hop]] == COST_INFINITY)) {

          if (state->costs[next_hop][node] == (state->costs[current_node][node] + 1))  {
            link_cost = COST_INFINITY;
            set_route(node, current_node, link_cost);
            next_hop = -1;
          }

          state->next_hops[current_node][node] = next_hop;
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
        s->next_hops[i][j] = -1;
        if (i == j) s->costs[i][j] = 0;
      }
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
        for (node_t i = 0; i < MAX_NODES; i++) {
          for (node_t j = 0; j < MAX_NODES; j++) {
            m->routes[i][j] = -1;
          }
        }
        for (node_t i = 0; i < MAX_NODES; i++) {
          m->dv[i] = s->costs[get_current_node()][i];

          int hop = 0;
          int best_hop = s->next_hops[get_current_node()][i];
          m->routes[i][hop] = best_hop;


          while (best_hop != -1) {
            best_hop = s->next_hops[best_hop][i];
            m->routes[i][hop++] = best_hop;
          }
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

  message_t* m = ((message_t*)message);
  for (node_t i = 0; i < MAX_NODES; i++) {
    s->costs[sender][i] = m->dv[i];

    s->next_hops[sender][i] = m->routes[i][0];
    int hop = 1;
    while (m->routes[i][hop-1] != -1 && hop < MAX_NODES) {
      s->next_hops[m->routes[i][hop-1]][i] = m->routes[i][hop];
      hop++;
    }
  }
  int send_msg = bellman_ford(s, get_current_node(), sender, last_cost);

  if(send_msg) {
    for(node_t node =  get_first_node(); node <= get_last_node(); node++) {
      if(get_link_cost(node) < COST_INFINITY && node != get_current_node()){
        message_t *m = (message_t*)malloc(sizeof(message_t));
        for (node_t i = 0; i < MAX_NODES; i++)
          m->dv[i] = s->costs[get_current_node()][i];
        send_message(node,m);
      }
    }
  }
}
