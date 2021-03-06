#include "cost_layer.h"
#include "utils.h"
#include "cuda.h"
#include "blas.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

COST_TYPE get_cost_type(char *s)
{
    if (strcmp(s, "sse")==0) return SSE;
    if (strcmp(s, "masked")==0) return MASKED;
    fprintf(stderr, "Couldn't find activation function %s, going with SSE\n", s);
    return SSE;
}

char *get_cost_string(COST_TYPE a)
{
    switch(a){
        case SSE:
            return "sse";
        case MASKED:
            return "masked";
    }
    return "sse";
}

cost_layer make_cost_layer(int batch, int inputs, COST_TYPE cost_type)
{
    fprintf(stderr, "Cost Layer: %d inputs\n", inputs);
    cost_layer l = {0};
    l.type = COST;

    l.batch = batch;
    l.inputs = inputs;
    l.outputs = inputs;
    l.cost_type = cost_type;
    l.delta = calloc(inputs*batch, sizeof(float));
    l.output = calloc(1, sizeof(float));
    #ifdef GPU
    l.delta_gpu = cuda_make_array(l.delta, inputs*batch);
    #endif
    return l;
}

void forward_cost_layer(cost_layer l, network_state state)
{
    if (!state.truth) return;
    if(l.cost_type == MASKED){
        int i;
        for(i = 0; i < l.batch*l.inputs; ++i){
            if(state.truth[i] == 0) state.input[i] = 0;
        }
    }
    copy_cpu(l.batch*l.inputs, state.truth, 1, l.delta, 1);
    axpy_cpu(l.batch*l.inputs, -1, state.input, 1, l.delta, 1);
    *(l.output) = dot_cpu(l.batch*l.inputs, l.delta, 1, l.delta, 1);
    //printf("cost: %f\n", *l.output);
}

void backward_cost_layer(const cost_layer l, network_state state)
{
    copy_cpu(l.batch*l.inputs, l.delta, 1, state.delta, 1);
}

#ifdef GPU

void pull_cost_layer(cost_layer l)
{
    cuda_pull_array(l.delta_gpu, l.delta, l.batch*l.inputs);
}

void push_cost_layer(cost_layer l)
{
    cuda_push_array(l.delta_gpu, l.delta, l.batch*l.inputs);
}

void forward_cost_layer_gpu(cost_layer l, network_state state)
{
    if (!state.truth) return;
    if (l.cost_type == MASKED) {
        mask_ongpu(l.batch*l.inputs, state.input, state.truth);
    }
    
    copy_ongpu(l.batch*l.inputs, state.truth, 1, l.delta_gpu, 1);
    axpy_ongpu(l.batch*l.inputs, -1, state.input, 1, l.delta_gpu, 1);

    cuda_pull_array(l.delta_gpu, l.delta, l.batch*l.inputs);
    *(l.output) = dot_cpu(l.batch*l.inputs, l.delta, 1, l.delta, 1);
}

void backward_cost_layer_gpu(const cost_layer l, network_state state)
{
    copy_ongpu(l.batch*l.inputs, l.delta_gpu, 1, state.delta, 1);
}
#endif

