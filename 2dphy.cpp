#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <SDL.h>

inline
float float_inverse(float num)
{
    return 1.0f / num;
}

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

struct Particle_Contact
{
    //Holds the particles that are involved in tge contact. The second of these
    //can be NULL, for constacts with the scenery.
    Particle* particle[2];
    //Holds the normal restitution coefficient at the contact.
    float restitution;
    //Holds the direction of the contact in world coordinates for first object perspective
    V2 contact_normal;
    //Holds the depth of penetration at the contact
    float penetration;
};

float calculate_separating_velocity(Particle_Contact* pc)
{
    V2 relative_velocity = pc->particle[0]->velocity;
    if(pc->particle[1]) relative_velocity -= pc->particle[1]->velocity;
    return v2_dot(relative_velocity, pc->contact_normal);
}

void contact_resolve_velocity(Particle_Contact* pc, float dt)
{
    //Find velocity in the direction of the contact
    float separating_velocity = calculate_separating_velocity(pc);
    
    //Check whether it needs to be resolved
    //The contact is either separating or stationary - there's no impulse required
    if(separating_velocity > 0) return;
    
    //Calculate the new separating velocity
    float new_sep_velocity = -separating_velocity * pc->restitution;
    
    V2 acc_caused_velocity = pc->particle[0]->acceleration;
    if(pc->particle[1]) acc_caused_velocity -= pc->particle[1]->acceleration;
    float acc_caused_sep_velocity = v2_dot(acc_caused_velocity, pc->contact_normal) * dt;
    
    //If we got a closing velocity due to acceleration build-up, remove it from the new separating velocity
    if(acc_caused_sep_velocity < 0)
    {
        new_sep_velocity += pc->restitution * acc_caused_sep_velocity;
        //Make sure we have't remove more than was
        if(new_sep_velocity < 0) new_sep_velocity = 0;
    }

    float delta_velocity  = new_sep_velocity - separating_velocity;

    //We apply the chage in velocity to each object in proportion to its inverse mass
    float total_inverser_mass = pc->particle[0]->inverse_mass;
    if(pc->particle[1]) total_inverser_mass += pc->particle[1]->inverse_mass;
    
    //If all particles have infinite mass, then impulses have no effect.
    if(total_inverser_mass <= 0) return;
    
    //Calculate the impulse to apply
    float impulse = delta_velocity / total_inverser_mass;

    //Find tge amount of impulse per unit of inverse mass
    V2 impulse_per_imass =  pc->contact_normal * impulse;

    //Apply impulses: they are applied in the direction of the contact, adn are proportional to the inverse mass.
    pc->particle[0]->velocity += (impulse_per_imass * pc->particle[0]->inverse_mass);
    if(pc->particle[1])
    {
        //Particle 1 goes in the oposite direction.
        pc->particle[1]->velocity += (impulse_per_imass * -pc->particle[1]->inverse_mass);
    }
}

void contact_resolve_interpenetrarion(Particle_Contact* pc, float dt)
{
    (void)dt;
    //If we dont have any penetration , skip this step
    if(pc->penetration <= 0) return;
    //The movement of each object is based on its inverse mass
    float total_inverser_mass = pc->particle[0]->inverse_mass;
    if(pc->particle[1]) total_inverser_mass += pc->particle[1]->inverse_mass;
    //If all particles have infinite inverse mass, then we do nothing
    if(total_inverser_mass == 0) return;
    //Find the amount of penetration resolution per unit of inverse mass
    V2 move_per_inverse_mass = pc->contact_normal * (-pc->penetration / total_inverser_mass);
    //Apply the penetration resolution
    pc->particle[0]->position += (move_per_inverse_mass * pc->particle[0]->inverse_mass);
    if(pc->particle[1])
    {
        pc->particle[1]->position += (move_per_inverse_mass * pc->particle[1]->inverse_mass);
    }
}

void contact_resolve(Particle_Contact* pc, float dt)
{
    contact_resolve_velocity(pc, dt);
    contact_resolve_interpenetrarion(pc, dt);
}

struct Particle_Contact_Resolver
{
    //Holds the number of iteration allowed
    uint32_t iterations;
    //Perfornace traking value - actual number of iterations used
    uint32_t iterations_used;
    
    //Resolves a set of particle contacts for both penetration and velocity
    void resolve_contacts(Particle_Contact* contact_array, uint32_t num_contacts, float dt)
    {
        iterations_used = 0;
        while(iterations_used < iterations)
        {
            //Find tge contact with the largest closing velocity
            float max = 0;
            uint32_t max_index = num_contacts;
            for(uint32_t i = 0; i < num_contacts; ++i)
            {
                float sep_vel = calculate_separating_velocity(&contact_array[i]);
                if(sep_vel < max)
                {
                    max = sep_vel;
                    max_index = i;
                }
            }

            contact_resolve(&contact_array[max_index], dt);
            iterations_used++;
        }    
    }
};

//Links cinnect two particles together, generating a contact if they violate the
//constrains fo their link. Its us as base class for cables and rods
struct Particle_Link
{
    //Holds ther pair of particles that are connected by this link
    Particle* particle[2];

    //Return the current legth of the cable
    float current_length()
    {
        V2 relative_pos = particle[0]->position - particle[1]->position;
        return v2_length(relative_pos);
    }

    //Fills ther given contact structure with the contact neede to keep the link from
    //violating its contrains. The methods return the number of contact that have been written.
    virtual uint32_t fill_contact(Particle_Contact* contact, uint32_t limit) = 0;
};

//Cables link a pair if particles, generating a contact if they stray too far apart
struct Particle_Cable : public Particle_Link
{
    //Holds the maximum length of the cable
    float max_length;
    //Holds the restitution (bounciness) of the cable
    float restitution;

    //Fills the given contact structure with the contact needed to keep the cable from overextending
    virtual uint32_t fill_contact(Particle_Contact* contact, uint32_t limit)
    {
        (void)limit;
        //Find the length of the cable
        float length = current_length(); 
        //Check wheter we are overextended
        if(length < max_length) return 0;
        
        //Otherwise return the contact.
        contact->particle[0] = particle[0];
        contact->particle[1] = particle[1];

        //Calculate the normals
        V2 normal = particle[1]->position - particle[0]->position;
        normal = v2_normailize(normal);
        contact->contact_normal = normal;
        contact->penetration = length-max_length;
        contact->restitution = restitution;
            
        return 1;
    }
};

//Rods link a pait of particles, generating a contact if they sary too far apart or too close
struct Particle_Rod : public Particle_Link
{
    //Holds the lengh of the rod
    float length;

    //Fills the given structure with the contact neede to keep the rod extending or compressing
    virtual uint32_t fill_contact(Particle_Contact* contact, uint32_t limit)
    {
        (void)limit;
        //Find the length of the cable
        float current_len= current_length(); 
        //Chech wheter we overextended
        if(current_len == length)
        {
            return 0;
        }

        //Otherwise return the contact
        contact->particle[0] = particle[0];
        contact->particle[1] = particle[1];

        //Calculate the normal
        V2 normal = particle[1]->position - particle[0]->position;
        normal = v2_normailize(normal);

        //The contact normal depends on whether we're extending or compressing
        if(current_len > length)
        {
            contact->contact_normal = normal;
            contact->penetration = current_len - length;
        }
        else
        {
            contact->contact_normal = normal * -1;
            contact->penetration = length - current_len;
        }

        //Always use zero restitution (no bounciness)
        contact->restitution = 0;

        return 1;
    }
};

//Keeps track of a set of particles, and provide the means to update them all
struct Particle_World
{
    //Holds one particle in the linked list of particles
    struct Particle_Registration
    {
        Particle* particle;
        Particle_Registration* next;
    };

    //Holds a list of the registrations
    Particle_Registration* first_particle;
    
    //Initializes the world for a simulation frame. This clears the foce accumulatorss for
    //particles in the world. After calling this, the particles can have their forces for this
    //frame added
    void start_frame()
    {
        Particle_Registration* reg = first_particle;
        while(reg)
        {
            //Remove all forces from the accumulator
            reg->particle->force_accum = {};
            //Get the next registration
            reg = reg->next;
        }
    }

    struct Particle_Contact_Generator
    {
        //Fills the given contact structure with the generated contact. The contact pointer should
        //point to the first available contact in a contact array, where limit is the maximum number
        //of contact in the array that can be written to. The method returns the number of contacts
        //that have been written
        virtual uint32_t add_contact(Particle_Contact* contact, uint32_t limit) = 0;
    };

    //Holds the force generators for the particles in this world
    Particle_Force_Registry registry;
    //Holds the resolver for contacts
    Particle_Contact_Resolver resolver;
    //Holds one registered contact generator
    struct Contact_Gen_Registration
    {
        Particle_Contact_Generator* gen;
        Contact_Gen_Registration* next;
    };
    
    //Holds a list of contact generators
    Contact_Gen_Registration* first_contact_gen;

    //Holds a list of contacts
    Particle_Contact* contacts; 
    //Holds the maximum number of contact allowed
    uint32_t max_contacts;
    bool calculate_iterations = true;


    //Calls each of the registered contact generators to report ther contacrs.
    //Retrun the number of generated contacts.
    uint32_t generate_contacts()
    {
        uint32_t limit = max_contacts;
        Particle_Contact* next_contact = contacts;
        
        Contact_Gen_Registration* reg = first_contact_gen;
        while(reg)
        {
            uint32_t used = reg->gen->add_contact(next_contact, limit);
            limit -= used;
            next_contact += used;
            //We run out of contacts to fill. This meanss we're missing contacts
            if(limit <= 0) break;
            reg = reg->next;
        }

        return max_contacts - limit;
    }
    //Integrate all the particles in this world forward in time by the given duration
    void integrate(float dt)
    {
        Particle_Registration* reg = first_particle;
        while(reg)
        {
            //Remove all forces from the accumulator
            integrate_particle_position(reg->particle, dt);
            //Get the next registration
            reg = reg->next;
        }
    }
    //Processes all the physics for the particle world
    void run_physics(float dt)
    {
        //Fist apply the forces generators
        registry.update_forces(dt);
        //Then integrate the objects
        integrate(dt);
        //Generate Contacts
        uint32_t used_contacts = generate_contacts();
        //And Process them
        if(calculate_iterations) resolver.iterations = (used_contacts * 2);
        resolver.resolve_contacts(contacts, used_contacts, dt);
    }
};

