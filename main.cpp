#include "2dphy.cpp"

#define WINDOW_TITLE "2dphy_Test_Bed"
#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define BACKGROUND_COLOR 0xFF3333AA 
#define HEX_TO_SDL(color) \
    (color >> 16 & 0xFF), \
    (color >> 8  & 0xFF), \
    (color >> 0  & 0xFF), \
    (color >> 24 & 0xFF)  \


V2 world_to_screen(V2 pos)
{
    float meters_to_pixels = 30.0f;
    return V2(pos.x*meters_to_pixels + (WINDOW_WIDTH/2),
            ((float)WINDOW_HEIGHT - (pos.y*meters_to_pixels+WINDOW_HEIGHT/2)));
}

struct Sim_State
{
    Particle_World world;
};

void init_physics_sim(Sim_State* sim_state)
{
    (void)sim_state;
}

void update_physics_sim(float dt, Sim_State* sim_state)
{
    sim_state->world.run_physics(dt);
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
        
        sim_state.world.start_frame();

        SDL_SetRenderDrawColor(renderer, HEX_TO_SDL(BACKGROUND_COLOR));
        SDL_RenderClear(renderer);
        
        //Render physics simulation here:
        render_physics_sim(renderer, &sim_state);
        update_physics_sim(0.016f, &sim_state);

        SDL_RenderPresent(renderer);
    }

    return 0;
}
