//Particle Forece Generator (OOP)
struct Particle_Force_Generator
{
    //Interface to calculate and update the force applied to the given particle
    virtual void update_force(Particle* particle, float dt) = 0;
};

#define MAX_FORCE_REGISTRATIONS 1024
//Holds all the force generators and the particles they apply to.
struct Particle_Force_Registry
{
    //IMPORTANT! not acess implementation only

    //Keeps track of one force generator and the particle it applies to
    struct Particle_Force_Registration
    {
        Particle* particle;
        Particle_Force_Generator* fg;
    };

    //List of registration
    Particle_Force_Registration registrations[MAX_FORCE_REGISTRATIONS];
    uint32_t registration_index = 0;

    //-----------------------------------------
    
    //Register the given force generator
    void add(Particle* particle, Particle_Force_Generator* fg)
    {
        assert(registration_index < MAX_FORCE_REGISTRATIONS);
        registrations[registration_index] = {particle, fg};
        registration_index++;
    }
    //Calls all the force generetors to update the forces of their corresponding particle
    void update_forces(float dt)
    {
        uint32_t total_registration = registration_index+1;
        for(uint32_t i = 0; i < total_registration; ++i)
        {
            registrations[i].fg->update_force(registrations[i].particle, dt);
        }
    }
};

//A Gravity force generator
struct Particle_Gravity : public Particle_Force_Generator
{
    //Holds the acceleration due to gravity
    V2 gravity;
    void update_force(Particle* particle, float dt)
    {
        (void)dt;
        if(!particle->inverse_mass) return;
        particle->force_accum += gravity * (1/particle->inverse_mass);
    }
};

//A Drag force generetor
struct Particle_Drag : public Particle_Force_Generator
{
    float k1;
    float k2;

    void update_force(Particle* particle, float dt)
    {
        (void)dt;
        V2 force = particle->velocity;

        //Calculate the total drag velocity
        float drag_coeff = v2_length(force);
        drag_coeff = k1 * drag_coeff + k2 * drag_coeff * drag_coeff;
        //Calculate the final force and apply it
        force = v2_normailize(force);
        force *= -drag_coeff;
        particle->force_accum += force;
    }
};
