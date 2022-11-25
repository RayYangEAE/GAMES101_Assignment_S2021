#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D pos = start;
        Vector2D stp = (end - start) / (num_nodes - 1);
        for (int i = 0; i < num_nodes; i++) { 
            masses.push_back(new Mass(pos, node_mass, false));
            pos += stp; 
        }

        for (int i = 0; i < num_nodes - 1; i++) {
            springs.push_back(new Spring(masses[i], masses[i + 1], k));
        }

        //Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D dir = s->m2->position - s->m1->position; 
            float dist = dir.norm();  //Vector2D force = s->k * dir;
            dir = dir.unit();
            Vector2D force = s->k * dir * (dist - s->rest_length); 
            s->m1->forces += force;
            s->m2->forces -= force;

            Vector2D rela_v = s->m2->velocity - s->m1->velocity;
            float kd = 0.1;
            Vector2D internal_damping = kd * dot(dir, rela_v) * dir;
            s->m1->forces += internal_damping;
            s->m2->forces -= internal_damping;
        }

        float global_damping = 0.005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity; 

                // TODO (Part 2): Add global damping
                m->forces -= global_damping * m->velocity;
                Vector2D a = m->forces / m->mass;
                //Vector2D step = m->velocity * delta_t; //for explicit method
                Vector2D step = m->velocity * delta_t + a * delta_t * delta_t; //for semi-implicit method
                
                m->position += step;
                m->velocity += a * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D dir = s->m2->position - s->m1->position;
            float dist = dir.norm();
            dir = dir.unit();
            Vector2D force = s->k * dir * (dist - s->rest_length);
            s->m1->forces += force;
            s->m2->forces -= force;

            Vector2D rela_v = s->m2->velocity - s->m1->velocity;
            float kd = 0.1;
            Vector2D internal_damping = kd * dot(dir, rela_v) * dir;
            s->m1->forces += internal_damping;
            s->m2->forces -= internal_damping;
        }

        float damping_factor = 0.00005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity;
                Vector2D a = m->forces / m->mass;
                
                // TODO (Part 4): Add global Verlet damping
                m->position = temp_position + (1 - damping_factor) * (temp_position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
