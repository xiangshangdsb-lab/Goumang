#pragma once

#include <stdbool.h>
#include <stddef.h>

/* Registers the v3.10.0 graph: topics, services, actions and parameters. */
void robot_services_init(void);

/* Map a parameter name to its protocol id; -1 when unknown. */
int robot_param_id_from_name(const char *name);

/* JSON array of {name,id,value,min,max,default}. */
size_t robot_params_json(char *buf, size_t cap);

/* Apply a parameter by name (validated by robot_bus). */
bool robot_param_set_by_name(const char *name, double value);