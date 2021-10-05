#include <stdint.h>
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

#define PLOT_POS    0x01
#define PLOT_VEL    0x02
#define PLOT_ACC    0x04
#define PLOT_ORB    0x08

struct plot {
    struct body *sat;   //body to consider as satellite
    struct body *ref;   //body to consider as center (ref)
    uint32_t plots; //bitmap of coordinates to plot
    uint32_t    nth; //skip steps
    char name[256];
    FILE *f;
};

struct state {
    struct body     *bodies;
    int             bcount;
    double          t;      //current time
    double          tmax;   //max sim duration
    double          dt;     //time step
    unsigned long   steps;  //step count
    struct plot     *plots;
    int             pcount;
};

struct state sim;

#define G 6.6743015E-11

/*---------------------------------------------------------------------------*/
int sim_init(struct state *dest) {
    dest->bodies = NULL;
    dest->bcount = 0;
    dest->plots = NULL;
    dest->pcount = 0;
    dest->tmax = 0;
    dest->dt = 0;
    return 0;
}

/*---------------------------------------------------------------------------*/
int sim_end(struct state *dest) {
    int p;
    for(p=0;p<dest->pcount;p++) {
        if(dest->plots[p].f) {
            fclose(dest->plots[p].f);
        }
    }
    free(dest->plots);
    free(dest->bodies);
    return 0;
}

/*---------------------------------------------------------------------------*/
int sim_start(struct state *dest) {
    int p;
    dest->t = 0;
    dest->steps = 0;
    for(p=0;p<dest->pcount;p++) {
        dest->plots[p].f = fopen(dest->plots[p].name,"wb");
        if(!dest->plots[p].f) {
            printf("failed to open plot%s\n", dest->plots[p].name);
        }
    }

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
int sim_body_add(struct state *dest, char *name, double mass, double radius) {
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
struct body *sim_body_find(struct state *dest, char *name) {
    int i;
    for(i=0;i<dest->bcount;i++) {
        if(!strcmp(dest->bodies[i].name, name)) {
            return &dest->bodies[i];
        }
    }
    return NULL;
}

/*---------------------------------------------------------------------------*/
int sim_plot_add(struct state *dest, char *file, struct body *sat, struct body *ref, uint32_t plots, uint32_t nth) {
    dest->plots = realloc(dest->plots, sizeof(struct plot) * (dest->pcount+1));
    if(dest->plots) {
        memset(&dest->plots[dest->pcount], 0, sizeof(struct plot));
        dest->plots[dest->pcount].sat   = sat;
        dest->plots[dest->pcount].ref   = ref;
        dest->plots[dest->pcount].plots = plots;
        dest->plots[dest->pcount].nth   = nth;
        strncpy(dest->plots[dest->pcount].name, file, 256);
        dest->pcount += 1;
        printf("sim: add plot file %s sat %s ref %s bits %08X\n",file,sat->name,ref->name,plots);
        return 0;
    }
    return 1; //failed
}

/*---------------------------------------------------------------------------*/
int sim_run(struct state *s) {
    uint32_t u,v;       //body indices
    double dx,dy,d2,d;  //distance stuff
    double f;           //force stuff
    //local vars for speedup related to cache proximity
    double ax,ay;       //accelerations
    double rux,ruy;     //central body position
    double um;          //central body mass

    //memory locality speedups:
    //load central body characteristics only once and not in inner iteration

    //compute forces on each body
    for(u = 0; u < s->bcount; u++) {
        um = s->bodies[u].mass;
        rux = s->bodies[u].rx;
        ruy = s->bodies[u].ry;
        ax = 0;
        ay = 0;
        for(v = 0; v < s->bcount; v++) {
            if(v==u) continue;
            dx = s->bodies[v].rx - rux;
            dy = s->bodies[v].ry - ruy;
            d2 = dx*dx + dy*dy;
            d = sqrt(d2);
            f  = G * um * s->bodies[v].mass / d2;
            //direction from v to u (normalized vector)
            dx = dx / d;
            dy = dy / d;
            //accumulate acceleration from v to u
            ax += dx * f / um;
            ay += dy * f / um;
        }
        //store central body acceleration only once
        s->bodies[u].ax = ax;
        s->bodies[u].ay = ay;
    }

    //integrate
    for(u = 0; u < s->bcount; u++) {
        s->bodies[u].vx += s->bodies[u].ax * s->dt;
        s->bodies[u].vy += s->bodies[u].ay * s->dt;
        s->bodies[u].rx += s->bodies[u].vx * s->dt;
        s->bodies[u].ry += s->bodies[u].vy * s->dt;
    }

    //detect collisions
    for(u = 0; u < s->bcount; u++) {
        for(v = u+1; v < s->bcount; v++) {
            dx = s->bodies[v].rx - s->bodies[u].rx;
            dy = s->bodies[v].ry - s->bodies[u].ry;
            d2 = dx*dx + dy*dy;
            d = sqrt(d2);
            if(d < (s->bodies[u].radius + s->bodies[v].radius)) {
                printf("collision!\n");
                s->t = s->tmax;
            }
        }
    }

    //do it
    s->t += s->dt;
    s->steps += 1;
    return 0;
}

/*---------------------------------------------------------------------------*/
int sim_plots(struct state *s) {
    int p;
    uint32_t pl;
    double drx,dry,dvx,dvy,dax,day,mu;


    for(p = 0; p<s->pcount; p++) {
        if(s->steps % s->plots[p].nth) continue;

        //printf("[%lu] %g ", s->steps, s->t - s->dt);
        fprintf(s->plots[p].f, "%g ", s->t);

        drx = s->plots[p].sat->rx - s->plots[p].ref->rx;
        dry = s->plots[p].sat->ry - s->plots[p].ref->ry;
        dvx = s->plots[p].sat->vx - s->plots[p].ref->vx;
        dvy = s->plots[p].sat->vy - s->plots[p].ref->vy;
        dax = s->plots[p].sat->ax - s->plots[p].ref->ax;
        day = s->plots[p].sat->ay - s->plots[p].ref->ay;
        mu = G * s->plots[p].ref->mass;
        pl=s->plots[p].plots;
        if(pl & PLOT_POS) {
            fprintf(s->plots[p].f, "%g %g ",drx,dry);
        }
        if(pl & PLOT_VEL) {
            fprintf(s->plots[p].f, "%g %g ",dvx,dvy);
        }
        if(pl & PLOT_ACC) {
            fprintf(s->plots[p].f, "%g %g ",dax,day);
        }
        if(pl & PLOT_ORB) {
            double d2,d,v2,v,h,ex,ey,e,a;

            //determine orbital parameters from state vector
            //https://downloads.rene-schwarz.com/download/M002-Cartesian_State_Vectors_to_Keplerian_Orbit_Elements.pdf

            //distance of sat to central body
            d2 = drx*drx + dry*dry;
            d = sqrt(d2);

            //orbital velocity
            v2 = dvx*dvx + dvy*dvy;
            v = sqrt(v2);

            //orbital momentum r = R cross V -> 2D means this is a single value along Z
            h = drx * dvy - dry * dvx;

            //eccentricity vector
            ex = ( dvy * h / mu) - drx / d;
            ey = (-dvx * h / mu) - dry / d;
            e = sqrt(ex * ex + ey * ey);

            //semimajor axis
            a = 1 / ((2/d)-(v2/mu));

            fprintf(s->plots[p].f, "%g %g %g %g ", d, v, e, a);
        }
        fprintf(s->plots[p].f, "\n");

    }
    return 0;
}

/*---------------------------------------------------------------------------*/
//[planet] name mass radius pos
int parse_planet(struct state *dest, char *buf) {
    double m,r;
    char *name,*ebuf;
    name = buf;
    printf("PLANET =>%s\n",buf);
    if(!*buf) {
        printf("missing planet name");
        return 1;
    }
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

    return sim_body_add(dest,name,m,r);
}

/*---------------------------------------------------------------------------*/
//[ship] name mass radius [around] planet alt angle orbspeed
int parse_ship(struct state *dest, char *buf) {
    double m,r;
    char *ship,*name,*ebuf;
    int ret;

    ship = buf;
    printf("SHIP =>%s\n",buf);
    if(!*buf) {
        printf("missing ship name");
        return 1;
    }
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
    r = strtod(buf,&ebuf);
    buf = ebuf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    ret = sim_body_add(dest,ship,m,r);
    if(ret) {
        printf("failed to add body %s\n", name);
        return 1;
    }

    //parse 'around' alt angle
    if(!*buf) {
        printf("missing ship pos method");
        return 1;
    }
    name = buf;
    while(*buf && *buf!=0x20) {
        buf += 1;
    }
    ebuf = buf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    *ebuf = 0;
    printf("pos method -> %s\n", name);

    if(!strcmp(name,"around")) {
        struct body *ref,*bship;
        double rad,spd;
        if(!*buf) {
            printf("missing body around to position");
            return 1;
        }
        name = buf;
        while(*buf && *buf!=0x20) {
            buf += 1;
        }
        ebuf = buf;
        while(*buf && *buf==0x20) {
            buf += 1;
        }
        if(!*buf) {
            printf("missing altitude around body");
            return 1;
        }
        *ebuf = 0;
        printf("pos ship around body: %s\n",name);
        ref = sim_body_find(dest,name);
        if(!ref) {
            printf("cannot find ref body: %s\n",name);
            return 1;
        }
        rad = strtod(buf,&ebuf);
        buf = ebuf;
        while(*buf && *buf==0x20) {
            buf += 1;
        }
        printf("at altitude %g\n",rad);
        rad += ref->radius;
        if(!*buf) {
            printf("missing angle around body");
            return 1;
        }
        r = strtod(buf,&ebuf);
        buf = ebuf;
        while(*buf && *buf==0x20) {
            buf += 1;
        }
        if(!*buf) {
            printf("missing orbital speed\n");
            return 1;
        }
        printf("at mean anomaly %g\n",r);
        spd = strtod(buf,&ebuf);
        buf = ebuf;
        while(*buf && *buf==0x20) {
            buf += 1;
        }
        printf("orbital velo %g\n",spd);
        bship = sim_body_find(dest,ship);
        bship->rx = ref->rx + rad * cos(r * M_PI / 180);
        bship->ry = ref->ry + rad * sin(r * M_PI / 180);
        printf("ship pos x=%g, y=%g\n",bship->rx, bship->ry);
        bship->vx = ref->vx + spd * cos((r+90) * M_PI / 180);
        bship->vy = ref->vy + spd * sin((r+90) * M_PI / 180);
        printf("ship vel x=%g, y=%g\n",bship->vx, bship->vy);
    }

    if(*buf) {
        printf("warning: spurious ship info: %s\n",buf);
    }

    return 0;
}

/*---------------------------------------------------------------------------*/
//[sim] timestep duration
int parse_sim(struct state *dest, char *buf) {
    double t,d;
    char *ebuf;
    printf("SIM =>%s\n",buf);
    if(!*buf) {
        printf("missing timestep");
        return 1;
    }
    t=strtod(buf,&ebuf);
    buf = ebuf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    if(!*buf) {
        printf("missing duration");
        return 1;
    }
    d=strtod(buf,&ebuf);
    buf = ebuf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    if(*buf) {
        printf("warning: spurious sim info: %s\n",buf);
    }
    dest->dt = t;
    dest->tmax = d;
    return 0;
}

/*---------------------------------------------------------------------------*/
//[plot] body ref param ... [pos,vel,acc,orb]
int parse_plot(struct state *dest, char *buf) {
    char *out, *sat, *ref, *par, *ebuf;
    struct body *bsat,*bref;
    uint32_t bits;
    unsigned long nth;

    printf("PLOT =>%s\n",buf);

    if(!*buf) {
        printf("missing sat name\n");
        return 1;
    }
    //find out end
    out = buf;
    while(*buf && *buf !=0x20) {
        buf += 1;
    }
    ebuf = buf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    *ebuf = 0;
    printf("out %s\n",sat);
    if(!*buf) {
        printf("missing sat name\n");
        return 1;
    }

    //find sat end
    sat = buf;
    while(*buf && *buf !=0x20) {
        buf += 1;
    }
    ebuf = buf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    *ebuf = 0;
    printf("sat %s\n",sat);
    if(!*buf) {
        printf("missing ref name\n");
        return 1;
    }

    //find ref
    ref = buf;
    //find ref end
    while(*buf && *buf !=0x20) {
        buf += 1;
    }
    ebuf = buf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    if(!*buf) {
        printf("no nth step\n");
        return 1;
    }
    *ebuf = 0;
    printf("ref %s\n",ref);

    //parse nthstep
    nth=strtol(buf,&ebuf,10);
    buf = ebuf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    printf("every nth %lu\n",nth);
    if(!*buf) {
        printf("no params\n");
        return 1;
    }
    printf("params %s\n",buf);

    bsat = sim_body_find(dest,sat);
    if(!bsat) {
        printf("satellite %s not found\n", sat);
        return 1;
    }

    bref = sim_body_find(dest,ref);
    if(!bref) {
        printf("reference body %s not found\n", ref);
        return 1;
    }
    bits = 0;

loop:
    par = buf;
    //find par end
    while(*buf && *buf !=0x20) {
        buf += 1;
    }
    ebuf = buf;
    while(*buf && *buf==0x20) {
        buf += 1;
    }
    *ebuf = 0;
    printf("par -> %s\n",par);
    if(!strcmp(par,"pos")) {
        bits |= PLOT_POS;
    } else if(!strcmp(par,"vel")) {
        bits |= PLOT_VEL;
    } else if(!strcmp(par,"acc")) {
        bits |= PLOT_ACC;
    } else if(!strcmp(par,"orb")) {
        bits |= PLOT_ORB;
    } else {
        printf("unknown param %s\n",par);
    }
    if(*buf) {
        goto loop;
    }
    return sim_plot_add(dest,out,bsat,bref,bits,nth);
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
    printf("config done\n");
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
        sim_plots(&sim);
    }
    sim_end(&sim);
    printf("simulation done\n");
    return 0;
}

