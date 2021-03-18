#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <SDL.h>

int rand_int(int min, int max)
{
    int shift =  max - min;
    int r = rand()%shift;
    return min + r; 
}

float rand_float(float min, float max)
{
    float num = (float)rand()/(float)RAND_MAX;
    num = num * (max - min) + min; 
    return num;
}

struct V2 
{
    float x;
    float y;

    V2(): x(0), y(0) {}
    V2(float x, float y): x(x), y(y) {}
};

V2 rand_v2(V2 min_velocity, V2 max_velocity)
{
    return V2(
            rand_float(min_velocity.x, max_velocity.x),
            rand_float(min_velocity.y, max_velocity.y));
}

inline
V2 operator+(V2 v0, V2 v1)
{
    return V2(v0.x+v1.x, v0.y+v1.y);
}

inline
void operator+=(V2& v0, V2 v1)
{
    v0 = v0 + v1;
}

inline
V2 operator-(V2 v0, V2 v1)
{
    return V2(v0.x-v1.x, v0.y-v1.y);
}

inline
void operator-=(V2& v0, V2 v1)
{
    v0 = v0 - v1;
}

inline 
V2 operator*(V2 v, float s)
{
    return V2(v.x*s, v.y*s);
}

inline 
V2 operator*(float s, V2 v)
{
    return V2(v.x*s, v.y*s);
}

inline
V2 operator*(V2 v0, V2 v1)
{
    return V2(v0.x*v1.x, v0.y*v1.y);
}

inline
void operator*=(V2& v, float s)
{
    v = v * s;
}

inline
float v2_dot(V2 v0, V2 v1)
{
    return v0.x*v1.x + v0.y*v1.y;
}

inline
V2 v2_perp(V2 v)
{
    return V2(-v.y, v.x);
}

inline
float v2_length_sqrt(V2 v)
{
    return v2_dot(v, v);
}

inline
float v2_length(V2 v)
{
    return sqrtf(v2_length_sqrt(v));
}

inline
V2 v2_normailize(V2 v)
{
    float l = v2_length(v);
    return V2(v.x/l, v.y/l);
}

inline V2 v2_gravity()
{
    return V2(0.0f, -10.0f);
}

struct Particle
{
    //Holds the linear position of the particle in world space
    V2 position;
    //Holds the linear velocity of the particle in world space
    V2 velocity;
    //Holds the acceleration of the particle
    V2 acceleration;
    
    //Holds the accumulated force to be applied at the next simulation iteration only.
    //This value is zeroed at each integration step
    //TODO: Clear force_accum each integration step
    V2 force_accum;

    //Holds the amount of damping applied to linear motion.
    //Damping is required to remove energy added through numerical instability in the integrator
    float damping;

    //Holds the inverser of the mass of the particle.
    //Its more useful to hold the inverse mass because in real time simulations it is more useful
    //to have objects with infinite mass (immovables) than zero mass
    float inverse_mass;

};

void integrate_particle_position(Particle* particle, float dt)
{
    assert(dt > 0);
    //Update linear position
    particle->position += (particle->velocity * dt);
    //Work out the acceleration from the force
    V2 resulting_acc = particle->acceleration;
    resulting_acc += (particle->force_accum * particle->inverse_mass);
    //Update Linear velocity from acceleration
    particle->velocity += (resulting_acc * dt);
    particle->velocity *= powf(particle->damping, dt);

    particle->force_accum = {};
}

#if 1
//Particle Forece Generator (NO OOP)
enum Particle_Force_Header
{
    GRAVITY = 0,
    DRAG = 1,
    TEST = 2,
    SPRING = 3,
    ANCHORED_SPRING = 4,
    BUNGEE = 5,
    BUOYANCY = 6,
};

struct Particle_Test
{
    //Must be first
    const Particle_Force_Header header = TEST;
    
    const char* text = "testing Particle_Force_Header cast!\n";
};

struct Particle_Gravity
{
    //Must be first
    const Particle_Force_Header header = GRAVITY; 
    
    V2 gravity;
};

struct Particle_Drag
{
    //Must be first
    const Particle_Force_Header header = DRAG;
    
    float k1;
    float k2;
};

struct Particle_Spring
{
    //Must be first
    const Particle_Force_Header header = SPRING;
   
    //The particle at the other end of the spring
    Particle* other; 
    float spring_constant;
    float rest_length; 
};

struct Particle_Anchored_Spring
{
    const Particle_Force_Header header = ANCHORED_SPRING;

    //The location of the anchored end of the spring
    V2* anchor;
    float spring_constant;
    float rest_length;
};

struct Particle_Bungee
{
    const Particle_Force_Header header = BUNGEE;
   
    //The particle at the other end of the spring
   Particle* other;
   float spring_constant;
   float rest_length;
     
};

struct Particle_Buoyancy
{
    //Force generator that applies a buoyancy force for a liquit paralel to x axies
    const Particle_Force_Header header = BUOYANCY;

    //Maximum submersion depth of the object before it generates its maximum buoyancy force
    float max_depth;
    //The volume of the object
    float volume;
    //The height of the water above y = 0
    float water_height;
    //The density if the liquid. Pure water has density of 1000 kg per cubic meter
    float liquid_density = 1000.0f; 
};

void update_particle_force_generator(void* force_pack, Particle* particle)
{
    Particle_Force_Header header = *(Particle_Force_Header*)force_pack;
    switch(header)
    {
        case GRAVITY:
        {
            Particle_Gravity* gravity_force = (Particle_Gravity*)force_pack; 
            if(!particle->inverse_mass) return;
            particle->force_accum += gravity_force->gravity * (1/particle->inverse_mass);
            printf("gravity force:(%f, %f)\n", gravity_force->gravity.x, gravity_force->gravity.y);
        }break;
        case DRAG:
        {
            Particle_Drag* drag_force = (Particle_Drag*)force_pack; 
            V2 force = particle->velocity;
            //Calculate the total drag velocity
            float drag_coeff = v2_length(force);
            drag_coeff = drag_force->k1 * drag_coeff + drag_force->k2 * drag_coeff * drag_coeff;
            //Calculate the final force and apply it
            force = v2_normailize(force);
            force *= -drag_coeff;
            particle->force_accum += force;
            printf("drag_coeff: %f\n", drag_coeff);
        }break;
        case TEST:
        {
            Particle_Test* test_force = (Particle_Test*)force_pack; 
            printf("%s", test_force->text);
        }break;
        case SPRING:
        {
            Particle_Spring* spring_force = (Particle_Spring*)force_pack; 
            //Calculate the vector of the srping
            V2 force = particle->position;
            force -= spring_force->other->position;
            //Calculate the magnitude of the force
            float magnitude = v2_length(force);
            magnitude = fabs(magnitude - spring_force->rest_length);
            magnitude *= spring_force->spring_constant;
            //Calculate final force and apply it
            force = v2_normailize(force);
            force *= magnitude;
            particle->force_accum += force;
        }break;
        case ANCHORED_SPRING:
        {
            Particle_Anchored_Spring* anchored_spring_force = (Particle_Anchored_Spring*)force_pack; 
            //Calculate the vector of the srping
            V2 force = particle->position;
            force -= *anchored_spring_force->anchor;
            //Calculate the magnitude of the force
            float magnitude = v2_length(force);
            magnitude = fabs(magnitude - anchored_spring_force->rest_length);
            magnitude *= anchored_spring_force->spring_constant;
            //Calculate the final force and apply it
            force = v2_normailize(force);
            force *= magnitude;
            particle->force_accum += force;
        }break;
        case BUNGEE:
        {
            Particle_Bungee* bungee_force = (Particle_Bungee*)force_pack;
            //Calculate the vector of the srping
            V2 force = particle->position;
            force -= bungee_force->other->position;
            //Calculate the magnitud of the force
            float magnitude = v2_length(force);
            if(magnitude <= bungee_force->rest_length) return;
            //Calculate the magnitud of the force
            magnitude = bungee_force->spring_constant * (bungee_force->rest_length - magnitude);
            //Calculate the final force and apply it
            force = v2_normailize(force);
            force *= -magnitude;
            particle->force_accum += force;
        
        }break;
        case BUOYANCY:
        {
            Particle_Buoyancy* buoyancy_force = (Particle_Buoyancy*)force_pack; 
            //Calculate the sumersion depth
            float depth = particle->position.y;
            //Check if we are out of the water
            if(depth >= buoyancy_force->water_height + buoyancy_force->max_depth) return;
            V2 force = {};
            //Chech if we are at maximum depth
            if(depth <= buoyancy_force->water_height - buoyancy_force->max_depth)
            {
                force.y = buoyancy_force->liquid_density * buoyancy_force->volume;
                particle->force_accum += force;
                return;
            }
            //Otherwise we are parlty sumbmerged
            force.y = buoyancy_force->liquid_density * buoyancy_force->volume * 
                (depth - buoyancy_force->max_depth - buoyancy_force->water_height) / 2 * buoyancy_force->max_depth;
            particle->force_accum += force;
        }break;
    }
}
//TODO: Maybe make a dinamic array to hold force_registrations
#define MAX_FORCE_REGISTRATIONS 1024
//Holds all the force generators and the particles they apply to.
struct Particle_Force_Registry
{
    //IMPORTANT! not access implementation only
    //Keeps track of one force generator and the particle it applies to
    struct Particle_Force_Registration
    {
        Particle* particle;
        void* fg;
    };

    //List of registration
    Particle_Force_Registration registrations[MAX_FORCE_REGISTRATIONS];
    uint32_t registration_index = 0;

    //-----------------------------------------
    
    //Register the given force generator
    void add(Particle* particle, void* fg)
    {
        assert(registration_index < MAX_FORCE_REGISTRATIONS);
        registrations[registration_index] = {particle, fg};
        registration_index++;
    }
    //Calls all the force generetors to update the forces of their corresponding particle
    void update_forces(float dt)
    {
        (void)dt;
        uint32_t total_registration = registration_index;
        for(uint32_t i = 0; i < total_registration; ++i)
        {
            update_particle_force_generator(registrations[i].fg, registrations[i].particle);
        }
    }
};
#else
#include "force_gen.cpp"
#endif

#define WINDOW_TITLE "2dphy"
#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define BACKGROUND_COLOR 0xFF3333AA 
#define HEX_TO_SDL(color) \
    (color >> 16 & 0xFF), \
    (color >> 8  & 0xFF), \
    (color >> 0  & 0xFF), \
    (color >> 24 & 0xFF)  \

inline
float float_inverse(float num)
{
    return 1.0f / num;
}

V2 world_to_screen(V2 pos)
{
    float meters_to_pixels = 30.0f;
    return V2(
            pos.x*meters_to_pixels + (WINDOW_WIDTH/2),
            ((float)WINDOW_HEIGHT - (pos.y*meters_to_pixels+WINDOW_HEIGHT/2)));
}

struct Sim_State
{

    Particle test_particle;
    Particle test_particle1;

    //Force Registry
    Particle_Force_Registry registry;

    //Testing forces
    Particle_Gravity gravity;
    Particle_Drag drag;
    Particle_Test test;

    //Spring forces
    //Force generator creates force for only one object. If we want to link two objects
    //with a spring, then we will need to create and register a generator for each
    Particle_Spring spring;
    Particle_Spring spring1;
};

void init_physics_sim(Sim_State* sim_state)
{
    //Init particles
    sim_state->test_particle.velocity = V2(10, 5);
    sim_state->test_particle.inverse_mass = 0.5;
    
    //Init test forces
    sim_state->gravity.gravity= V2(5, 2);
    sim_state->drag.k1 = 1.0f;
    sim_state->drag.k2 = 0.2;
    
    //Init spring forces 
    sim_state->spring.other = &sim_state->test_particle1;
    sim_state->spring.spring_constant = 1.0f;
    sim_state->spring.rest_length = 2.0f;

    sim_state->spring1.other = &sim_state->test_particle;
    sim_state->spring1.spring_constant = 1.0f;
    sim_state->spring1.rest_length = 2.0f;

    //Add forces to registry
    sim_state->registry.add(&sim_state->test_particle, &sim_state->gravity);
    sim_state->registry.add(&sim_state->test_particle, &sim_state->drag);
    sim_state->registry.add(&sim_state->test_particle, &sim_state->test);
    sim_state->registry.add(&sim_state->test_particle, &sim_state->spring);
    sim_state->registry.add(&sim_state->test_particle1, &sim_state->spring1);

}

void update_physics_sim(float dt, Sim_State* sim_state)
{
    (void)dt;
    (void)sim_state;
    sim_state->registry.update_forces(dt);
    printf("-----------------------------\n\n");
}

void render_physics_sim(SDL_Renderer* renderer, Sim_State* sim_state)
{
    (void)sim_state;
    (void)renderer;
}

int main(int argc, char** argv)
{
    (void) argc;
    (void) argv;
    
    SDL_Window* window = SDL_CreateWindow(
            WINDOW_TITLE, 
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            WINDOW_WIDTH,
            WINDOW_HEIGHT,
            SDL_WINDOW_SHOWN);
    
    SDL_Renderer* renderer = SDL_CreateRenderer(
            window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    Sim_State sim_state = {}; 
    init_physics_sim(&sim_state);

    bool quit = false;
    while(!quit)
    {
        //Process all event from the OS
        SDL_Event event;
        while(SDL_PollEvent(&event))
        {
            switch(event.type)
            {
                case SDL_QUIT: 
                {
                    quit = true;
                }break;
            }
        }
        //Update phusics simulation here:
        //TODO: Get real delta time of each frame
        update_physics_sim(0.016f, &sim_state);

        SDL_SetRenderDrawColor(renderer, HEX_TO_SDL(BACKGROUND_COLOR));
        SDL_RenderClear(renderer);
        
        //Render physics simulation here:
        render_physics_sim(renderer, &sim_state);

        SDL_RenderPresent(renderer);
    }

    return 0;
}
