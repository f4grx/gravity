#include <math.h>
#include <string.h>
#include <stdio.h>

/*2d gravity*/

struct body {
    double mass;    //kilograms
    double radius;  //meters, circular body
    double pos_x;   // meters
    double pos_y;   // meters
    double speed_x; // m/sec
    double speed_y; // m/sec
    double accel_x;
    double accel_y;
};

// fixed earth
const struct body earth = {
    5.97237E24, //mass in kg
    6.371E6,    //radius in m
    0,0,        //pos
    0,0,        //speed
};

//iss (approximate)
const struct body iss = {
    417289,     //417T
    110,        //largest dimension 110m
    0, 325000+earth.radius,  //325km alt MSL
    7666, 0,    //27600km/h = 7666m/s
};

struct body bodies[2];
#define central 0
#define sat 1

#define G 6.6743015E-11

#define dt 1e-3
#define time 8000


int main(int argc, char **argv) {
    double t=0;
    unsigned long i = 0;

    double d2,d,f,dx,dy,vx,vy,h,ex,ey,mu,e,a,s2,s;
    int nbodies = 2;

    int u,v;

    memcpy(&bodies[central], &earth, sizeof(struct body));
    memcpy(&bodies[sat]    , &iss  , sizeof(struct body));
    
    mu = G * bodies[central].mass;

again:
    //printf("i=%lu t=%g\n",i,t);
    //compute forces on each body
    for(u = 0; u < nbodies; u++) {
        //printf("gravitational forces on body %d\n",u);
        bodies[u].accel_x = 0;
        bodies[u].accel_y = 0;

        for(v = 0; v < nbodies; v++) {
            if(v==u) continue;
            dx = bodies[v].pos_x - bodies[u].pos_x;
            dy = bodies[v].pos_y - bodies[u].pos_y;
            d2 = dx*dx + dy*dy;
            d = sqrt(d2);
            f  = G * bodies[u].mass * bodies[v].mass / d2;
            //direction from v to u (normalized vector)
            dx = dx / d;
            dy = dy / d;
            //acceleration from v to u
            dx = dx * f / bodies[u].mass;
            dy = dy * f / bodies[u].mass;
            //printf(" from body %d dist=%g m, f=%g N ax=%g ay=%g\n",v,d,f,dx,dy);
            bodies[u].accel_x += dx;
            bodies[u].accel_y += dy;
        }
    }
    //integrate
    for(u = 0; u < nbodies; u++) {
        bodies[u].speed_x += bodies[u].accel_x * dt;
        bodies[u].speed_y += bodies[u].accel_y * dt;
        bodies[u].pos_x   += bodies[u].speed_x * dt;
        bodies[u].pos_y   += bodies[u].speed_y * dt;
    }

    //detect collisions
    for(u = 0; u < nbodies; u++) {
        for(v = u+1; v < nbodies; v++) {
            dx = bodies[v].pos_x - bodies[u].pos_x;
            dy = bodies[v].pos_y - bodies[u].pos_y;
            d2 = dx*dx + dy*dy;
            d = sqrt(d2);
            if(d < (bodies[u].radius + bodies[v].radius)) {
                printf("collision!\n");
                t = time;
            }
        }
    }

    //compute orbital parameters
    //https://downloads.rene-schwarz.com/download/M002-Cartesian_State_Vectors_to_Keplerian_Orbit_Elements.pdf

    //distance of sat to central body
    dx = bodies[sat].pos_x - bodies[central].pos_x;
    dy = bodies[sat].pos_y - bodies[central].pos_y;
    d2 = dx*dx + dy*dy;
    d = sqrt(d2);

    //orbital speed
    vx = bodies[sat].speed_x - bodies[central].speed_x;
    vy = bodies[sat].speed_y - bodies[central].speed_y;
    s2 = vx*vx + vy*vy;
    s = sqrt(s2);

    //orbital momentum r = R cross V -> 2D means this is a single value along Z
    h = dx * vy - dy * vx;

    //eccentricity vector
    ex = ( vy * h / mu) - dx / d;
    ey = (-vx * h / mu) - dy / d;
    e = sqrt(ex * ex + ey * ey);

    //semimajor axis
    a = 1 / ((2/d)-(s2/mu));

    if(!(i%10)) printf("%g %g %g %g %g\n", bodies[sat].pos_x, bodies[sat].pos_y, e, a, s);

    //iterate
    t = t + dt;
    i = i + 1;
    if(t < time) goto again;
    
}

