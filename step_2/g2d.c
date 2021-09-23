#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*2d gravity*/

#define NAMELEN 16
struct body {
    double  mass;       //kilograms
    double  radius;     //meters, circular body
    double  rx,ry;      //pos in meters
    double  vx,vy;      //speed in m/sec
    double  ax,ay;      //accels in m/sec^2
    char    name[NAMELEN];
};

struct state {
    struct body     *bodies;
    int             bcount;
    double          t;      //current time
    double          tmax;   //max sim duration
    double          dt;     //time step
    unsigned long   steps;  //step count
};

struct state sim;

#define G 6.6743015E-11

/*---------------------------------------------------------------------------*/
int sim_init(struct state *dest) {
    dest->bodies = NULL;
    dest->bcount = 0;
    dest->tmax = 0;
    dest->dt = 0;
    return 0;
}

/*---------------------------------------------------------------------------*/
int sim_end(struct state *dest) {
    free(dest->bodies);
    return 0;
}

/*---------------------------------------------------------------------------*/
int sim_start(struct state *dest) {
    dest->t = 0;
    dest->steps = 0;
    return 0;
}

/*---------------------------------------------------------------------------*/
int sim_check(struct state *dest) {
    printf("check: body count = %d\n", dest->bcount);
    if(!dest->bcount) return 1; //no bodies
    if(!dest->dt) return 1; //no timestep
    if(!dest->tmax) return 1; //no max duration
    return 0;
}

/*---------------------------------------------------------------------------*/
int sim_done(struct state *dest) {
    return (dest->t >= dest->tmax);
}

/*---------------------------------------------------------------------------*/
int sim_add_body(struct state *dest, char *name, double mass, double radius) {
    dest->bodies = realloc(dest->bodies, sizeof(struct body) * (dest->bcount+1));
    if(dest->bodies) {
        memset(&dest->bodies[dest->bcount], 0, sizeof(struct body));
        dest->bodies[dest->bcount].mass = mass;
        dest->bodies[dest->bcount].radius = radius;
        strncpy(dest->bodies[dest->bcount].name, name, NAMELEN);
        dest->bcount += 1;
        printf("sim: add body %s mass %g rad %g\n",name,mass,radius);
        return 0;
    }
    return 1; //failed
}

/*---------------------------------------------------------------------------*/
int sim_run(struct state *s) {
    printf("[%lu] %g\n", s->steps, s->t);

    //do it
    s->t += s->dt;
    s->steps += 1;
    return 0;
}

/*---------------------------------------------------------------------------*/
//[planet] name mass radius pos
int parse_planet(struct state *dest, char *buf) {
    double m,r;
    char *name,*ebuf;
    name = buf;
    printf("PLANET =>%s\n",buf);
    //find name end
    while(*buf && *buf !=0x20) {
        buf += 1;
    }
    ebuf = buf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    if(!*buf) {
        printf("missing planet mass");
        return 1;
    }
    *ebuf = 0; //cut name
    m=strtod(buf,&ebuf);
    buf = ebuf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    if(!*buf) {
        printf("missing planet radius");
        return 1;
    }
    r=strtod(buf,&ebuf);
    buf = ebuf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    if(*buf) {
        printf("warning: spurious planet info: %s\n",buf);
    }

    return sim_add_body(dest,name,m,r);
}

/*---------------------------------------------------------------------------*/
//[ship] name mass radius [around] planet alt angle
int parse_ship(struct state *dest, char *buf) {
    double m,r;
    char *name,*ebuf;
    name = buf;
    printf("SHIP =>%s\n",buf);
    //find name end
    while(*buf && *buf !=0x20) {
        buf += 1;
    }
    ebuf = buf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    if(!*buf) {
        printf("missing ship mass");
        return 1;
    }
    *ebuf = 0; //cut name
    m=strtod(buf,&ebuf);
    buf = ebuf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    if(!*buf) {
        printf("missing ship radius");
        return 1;
    }
    r=strtod(buf,&ebuf);
    buf = ebuf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    if(*buf) {
        printf("warning: spurious ship info: %s\n",buf);
    }

    return sim_add_body(dest,name,m,r);
}

/*---------------------------------------------------------------------------*/
//[sim] timestep duration
int parse_sim(struct state *dest, char *buf) {
    printf("SIM =>%s\n",buf);
    return 0;
}

/*---------------------------------------------------------------------------*/
//[plot] body param ref
int parse_plot(struct state *dest, char *buf) {
    printf("PLOT =>%s\n",buf);
    return 0;
}

/*---------------------------------------------------------------------------*/
int parse_line(struct state *dest, char *buf) {
    char *inst;

    inst = buf;

    //skip all until spaces
    while(*buf && *buf !=0x20) {
        buf += 1;
    }
    if(*buf) { //not end of line
        *buf = 0; //cut instruction
        buf += 1; //skip after
        //skip all spaces
        while(*buf && *buf==0x20) {
            buf += 1;
        }
    }

    if(!strcmp(inst,"planet")) {
        return parse_planet(dest, buf);
    } else if(!strcmp(inst,"ship")) {
        return parse_ship(dest, buf);
    } else if(!strcmp(inst,"sim")) {
        return parse_sim(dest, buf);
    } else if(!strcmp(inst,"plot")) {
        return parse_plot(dest, buf);
    } else {
        printf("unknown command : %s\n", inst);
        printf("params: %s\n", buf);
    }
    return 1;
}

/*---------------------------------------------------------------------------*/
int parse(struct state *dest, const char *fname) {
    FILE *f;
    char buf[256];
    char *ptr;
    int len;
    int ret;

    f = fopen(fname, "rb");
    if(!f) {
        printf("cant open: %s\n", fname);
        return 1;
    }

    while(!feof(f)) {
        ptr = fgets(buf, sizeof(buf), f);
        if(!ptr) {
            break;
        }
        len = strlen(ptr);

        if(ptr[len-1] == 0x0a) {
            ptr[len-1] = 0;
            len -= 1;
        }
        if(ptr[len-1] == 0x0d) {
            ptr[len-1] = 0;
            len -= 1;
        }
        while(*ptr && *ptr==0x20) {
            ptr++;
            len--;
        }
        if(!len) {
            continue;
        }
        if(ptr[0]=='#') {
            continue;
        }
        ret = parse_line(dest,ptr);
        if(ret) {
            break;
        }
    }

    fclose(f);
    printf("done\n");
    return 0;
}

/*---------------------------------------------------------------------------*/
int main(int argc, char **argv) {
    if(argc != 2) {
        printf("%s <simfile>\n", argv[0]);
        return 1;
    }
    sim_init(&sim);

    parse(&sim, argv[1]);

    if(sim_check(&sim)) {
        printf("simulation failed check\n");
        return 1;
    }

    printf("simulation start\n");
    sim_start(&sim);
    while(!sim_done(&sim)) {
        sim_run(&sim);
    }
    sim_end(&sim);
    printf("simulation done\n");
    return 0;
}

#if 0
int sim() {
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
#endif

