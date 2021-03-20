#include "2dphy.cpp"

//Fireworks
struct Firework : public Particle
{
    //Integer type used for firework rules
    uint32_t type;
    //Age of a firework determines when it denotates
    float age;
};

bool firework_update(Firework* firework, float dt)
{
    //Update the physical state
    integrate_particle_position(firework, dt);
    //Work backward from the age to zero
    firework->age -= dt;
    return (firework->age < 0);
}

struct Firework_Rule
{
    //The type of firework that is manage by this rule
    uint32_t type;
    //The minimun length of the fuse
    float min_age;
    //The maximun length of the fuse
    float max_age;
    //The minimun relative velocity of this firework
    V2 min_velocity;
    //The maximun relative velocity of this firework
    V2 max_velocity;
    //The damping of this firework type
    float damping;

    //The payload is the new firework to create when this firework fuse is over
    struct Payload
    {
        //The type of the new particle to create
        uint32_t type;
        //The number of particles in this payload
        uint32_t count;
    };

    //Number of payloads for this firework type
    uint32_t payload_count;
    //The set of payloads
    Payload* payloads;
};

void firework_rule_create(Firework_Rule rule, Firework* firework, Firework* parent = 0)
{
    firework->type = rule.type;
    firework->age = rand_float(rule.min_age, rule.max_age);
    if(parent) firework->position = parent->position;
    //The velocity is the particle velocity
    V2 vel = {};
    if(parent) vel = parent->velocity;
    vel += rand_v2(rule.min_velocity, rule.max_velocity);
    firework->velocity = vel;

    //We use mass of 1 in all cases
    firework->inverse_mass = 1;
    firework->damping = rule.damping;
    firework->acceleration = v2_gravity();

    firework->force_accum = {};
}

void firework_rule_paiload_create(Firework_Rule* rule, uint32_t number)
{
    rule->payloads = (Firework_Rule::Payload*)malloc(number*sizeof(Firework_Rule::Payload));
    rule->payload_count = number;
}

#define WINDOW_TITLE "2dphy_fireworks"
#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define BACKGROUND_COLOR 0xFF3333AA 
#define HEX_TO_SDL(color) \
    (color >> 16 & 0xFF), \
    (color >> 8  & 0xFF), \
    (color >> 0  & 0xFF), \
    (color >> 24 & 0xFF)  \

#define NUM_FIREWORKS_RULES 4
#define NUM_FIREWORKS 1024

V2 world_to_screen(V2 pos)
{
    float meters_to_pixels = 30.0f;
    return V2(
            pos.x*meters_to_pixels + (WINDOW_WIDTH/2),
            ((float)WINDOW_HEIGHT - (pos.y*meters_to_pixels+WINDOW_HEIGHT/2)));
}

struct Sim_State
{
    Firework_Rule rules[NUM_FIREWORKS_RULES];
    Firework fireworks[NUM_FIREWORKS];
    uint32_t next_firework = 0;
};

void create_firework(Sim_State* sim_state, uint32_t type, Firework* parent)
{
    //Get the rule needed to create this firework
    Firework_Rule* rule = sim_state->rules + (type - 1);
    Firework* firework = sim_state->fireworks + sim_state->next_firework;
    //Create the firework
    firework_rule_create(*rule, firework, parent);
    sim_state->next_firework = (sim_state->next_firework + 1) % NUM_FIREWORKS;
}

void create_firework(Sim_State* sim_state, uint32_t type, uint32_t number, Firework* parent)
{
    for(uint32_t i = 0; i < number; ++i)
    {
        create_firework(sim_state, type, parent);
    }
}

void init_physics_sim(Sim_State* sim_state)
{
    //Init fireworks
    for(uint32_t firework_index = 0; firework_index < NUM_FIREWORKS; ++firework_index)
    {
        Firework* firework = sim_state->fireworks + firework_index;
        firework->type = 0;

    }
    
#if 0 
    //Init fireworks rules
    Firework_Rule* rule0 = &sim_state->rules[0];
    rule0->type = 1;
    rule0->min_age = 0.5;
    rule0->max_age = 1.4f;
    rule0->min_velocity = V2(-5, 25);
    rule0->max_velocity = V2(5, 28);
    rule0->damping = 0.1f;
    firework_rule_paiload_create(rule0, 1);
    rule0->payloads[0] = {3, 15}; //Type and count

    Firework_Rule* rule1 = &sim_state->rules[1];
    rule1->type = 2;
    rule1->min_age = 0.5f;
    rule1->max_age = 1.0f;
    rule1->min_velocity = V2(-5, -10);
    rule1->max_velocity = V2(5, 10);
    rule1->damping = 0.8f;

    Firework_Rule* rule2 = &sim_state->rules[2];
    rule2->type = 3;
    rule2->min_age = 0.5f;
    rule2->max_age = 1.5f;
    rule2->min_velocity = V2(-5, -5);
    rule2->max_velocity = V2(5, 5);
    rule2->damping = 0.1f;
    firework_rule_paiload_create(rule2, 2);
    rule2->payloads[0] = {2, 5};
    rule2->payloads[1] = {4, 10};

    Firework_Rule* rule3 = &sim_state->rules[3];
    rule3->type = 4;
    rule3->min_age = 0.25f;
    rule3->max_age = 0.5f;
    rule3->min_velocity = V2(-10, 5);
    rule3->max_velocity = V2(10, 5);
    rule3->damping = 0.2f;
#else
    //Init fireworks rules
    Firework_Rule* rule0 = &sim_state->rules[0];
    rule0->type = 1;
    rule0->min_age = 0.5;
    rule0->max_age = 1.4f;
    rule0->min_velocity = V2(-5, 20);
    rule0->max_velocity = V2(5, 25);
    rule0->damping = 0.1f;
    firework_rule_paiload_create(rule0, 2);
    rule0->payloads[0] = {2, 15}; //Type and count
    rule0->payloads[1] = {4, 15}; //Type and count

    Firework_Rule* rule1 = &sim_state->rules[1];
    rule1->type = 2;
    rule1->min_age = 0.5f;
    rule1->max_age = 1.0f;
    rule1->min_velocity = V2(-7, -7);
    rule1->max_velocity = V2(7, 7);
    rule1->damping = 0.8f;
    firework_rule_paiload_create(rule1, 1);
    rule1->payloads[0] = {3, 5}; //Type and count

    Firework_Rule* rule2 = &sim_state->rules[2];
    rule2->type = 3;
    rule2->min_age = 0.5f;
    rule2->max_age = 1.5f;
    rule2->min_velocity = V2(-5, -5);
    rule2->max_velocity = V2(5, 5);
    rule2->damping = 0.1f;

    Firework_Rule* rule3 = &sim_state->rules[3];
    rule3->type = 4;
    rule3->min_age = 0.25f;
    rule3->max_age = 0.5f;
    rule3->min_velocity = V2(-11, -11);
    rule3->max_velocity = V2(11, 11);
    rule3->damping = 0.2f;
    firework_rule_paiload_create(rule3, 1);
    rule3->payloads[0] = {3, 5}; //Type and count

#endif
}

void update_physics_sim(float dt, Sim_State* sim_state)
{
    static float create_firework_timer = 0.0f;
    if(create_firework_timer > 2)
    {
        create_firework(sim_state, 1, 0);
        create_firework_timer = 0.0f;
    }
    create_firework_timer += dt;

    for(uint32_t firework_index = 0; firework_index < NUM_FIREWORKS; ++firework_index)
    {
        Firework* firework = sim_state->fireworks + firework_index;
        if(firework->type > 0)
        {
            //Does it need removing
            if(firework_update(firework, dt))
            {
                //Find the appropiate rule
                Firework_Rule* rule = sim_state->rules + (firework->type-1);
                firework->type = 0;

                //Add the payload
                for(uint32_t i = 0; i < rule->payload_count; ++i)
                {
                    Firework_Rule::Payload* payload = rule->payloads + i;
                    create_firework(sim_state, payload->type, payload->count, firework);
                }
            }
        }
        else
        {
            *firework = {};
        }
    }
}

void render_physics_sim(SDL_Renderer* renderer, Sim_State* sim_state)
{
    for(uint32_t firework_index = 0; firework_index < NUM_FIREWORKS; ++firework_index)
    {
        Firework* firework = sim_state->fireworks + firework_index;
        if(firework->type > 0)
        {
            switch(firework->type)
            {

                case 1:
                {
                    SDL_SetRenderDrawColor(renderer, HEX_TO_SDL(0xFFFF0000));
                }break;
                case 2:
                {
                    SDL_SetRenderDrawColor(renderer, HEX_TO_SDL(0xFF00FF00));
                }break;
                case 3:
                {
                    SDL_SetRenderDrawColor(renderer, HEX_TO_SDL(0xFFFF00FF));
                }break;
                case 4:
                {
                    SDL_SetRenderDrawColor(renderer, HEX_TO_SDL(0xFFFFFF00));
                }break;
            }
            SDL_Rect firework_rect;
            V2 screen_position = world_to_screen(firework->position);
            firework_rect.x = screen_position.x;
            firework_rect.y = screen_position.y;
            firework_rect.w = 5;
            firework_rect.h = 5;
            SDL_RenderFillRect(renderer, &firework_rect);
        }

    }
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
