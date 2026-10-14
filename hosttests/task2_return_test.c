#include "Task2Route.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define PI 3.14159265358979323846f

static Task2RoutePose Advance(Task2RoutePose p, float d, float r) {
    if(r) { float yaw=p.yawRad+d/r; p.xMm+=r*(sinf(yaw)-sinf(p.yawRad));
        p.yMm+=r*(cosf(p.yawRad)-cosf(yaw));p.yawRad=yaw; }
    else { p.xMm+=d*cosf(p.yawRad);p.yMm+=d*sinf(p.yawRad); }
    return p;
}
static Task2RouteReport Empty(void) {
    return (Task2RouteReport){.minimumObstacleGapMm=INFINITY,.minimumSideMarginMm=INFINITY,
        .minimumParkSideMarginMm=INFINITY,.minimumParkBackMarginMm=INFINITY};
}
static void Trace(Task2Route *route, Task2RoutePose p, float width, unsigned completed, bool home) {
    unsigned starts=0;
    for (;;) {
        assert(++starts<40);
        unsigned next=completed+1;
        if(next==route->stepCount)break;
        assert(next<route->stepCount);
        Task2RouteStep step=route->steps[next];
        Task2RouteConfig bounds=Task2Route_ReturnBounds(route);
        assert(bounds.parkSideWallBufferMm==100 && bounds.parkBackWallBufferMm==75);
        if(route->obstacle2IRDeparture && !home)bounds.obstacleBufferMm=1;
        Task2RoutePose initial=p;
        float length=fabsf(step.distanceMm);
        for(float d=0;d<=length;d+=0.5f) {
            Task2RoutePose sample=Advance(initial,copysignf(d,step.distanceMm),step.radiusMm);
            Task2RouteReport clearance=Empty();
            if(!Task2Route_CheckPose(&bounds,&sample,&clearance)) {
                fprintf(stderr,"clearance step %u d %.1f obstacle %.2f side %.2f park %.2f %.2f\n",next,d,
                    clearance.minimumObstacleGapMm,clearance.minimumSideMarginMm,
                    clearance.minimumParkSideMarginMm,clearance.minimumParkBackMarginMm);assert(0);
            }
        }
        p=Advance(initial,step.distanceMm,step.radiusMm);completed=next;
        if(route->returnFallbackActive && route->fallbackHomeStep>0 && completed+1==route->fallbackHomeStep) {
            home=true;assert(Task2Route_ReplanParkingReturn(route,&p,width,completed));
        }
        if(completed+1==route->stepCount) {
            if(route->obstacle2ParallelReturnPending)assert(Task2Route_ReplanParallelReturn(route,&p,width,completed));
            else if(route->obstacle2ParkReturnPending) {home=true;assert(Task2Route_ReplanParkingReturn(route,&p,width,completed));}
        }
    }
    assert(Task2Route_IsParked(&route->config,&p));
    assert(route->report.completeReturnCertified);
}
static Task2Route Setup(Task2RouteConfig c, bool ir, unsigned stage) {
    Task2Route route;memset(&route,0,sizeof(route));route.config=c;
    route.obstacle2IRDeparture=ir;route.obstacle2LateralDeparture=!ir;
    route.obstacle2ReturnPending=stage==0;route.obstacle2ParallelReturnPending=stage==1;
    route.obstacle2ParkReturnPending=stage==2;route.stepCount=stage==0?6:8;
    return route;
}
static void Recorded(void) {
    for(int mirror=-1;mirror<=1;mirror+=2) {
        Task2RouteConfig c=task2RouteConfig;c.firstArrow=(Task2Arrow)-mirror;c.secondArrow=(Task2Arrow)mirror;
        c.parkToObstacle1Mm=1046.96704f;
        Task2RoutePose p={1852.19568f,mirror*315.456848f,mirror*1.58878374f};
        Task2Route route=Setup(c,true,0);assert(Task2Route_ReplanObstacle2(&route,&p,552.776367f,5));
        assert(route.report.returnFallbackUsed && route.report.poseAdjustmentUsed);
        assert(route.steps[6].distanceMm<0);
        Trace(&route,p,552.776367f,5,false);
        printf("PASS run21 mirror %d: reverse %.0f mm, park buffers hard\n",mirror,-route.steps[6].distanceMm);
        c.parkToObstacle1Mm=1042.33643f;
        p=(Task2RoutePose){2037.50232f,-mirror*455.787964f,mirror*3.34525776f};
        route=Setup(c,true,2);assert(Task2Route_ReplanParkingReturn(&route,&p,514.770386f,7));Trace(&route,p,514.770386f,7,true);
        printf("PASS run16 rear-stop mirror %d\n",mirror);
    }
    for(int mirror=-1;mirror<=1;mirror+=2) {
        Task2RouteConfig c=task2RouteConfig;c.firstArrow=c.secondArrow=(Task2Arrow)mirror;
        Task2RoutePose p={1873.74036f,mirror*224.993179f,mirror*0.691187f};
        Task2Route route=Setup(c,false,0);assert(Task2Route_ReplanObstacle2(&route,&p,636.7f,5));Trace(&route,p,636.7f,5,false);
        printf("PASS run22 mirror %d\n",mirror);
    }
}
int main(void) {
    setbuf(stdout,NULL);Recorded();
    unsigned cases=0;
    const float widths[]={300,500,751,800};
    const float distances[]={600,1050,1500};
    for(unsigned w=0;w<4;++w)for(unsigned o=0;o<3;++o)for(unsigned g=0;g<3;++g)
    for(int mirror=-1;mirror<=1;mirror+=2)for(unsigned same=0;same<2;++same)
    for(int lateral=-1;lateral<=1;++lateral)for(int heading=-1;heading<=1;++heading) {
        Task2RouteConfig c=task2RouteConfig;c.firstArrow=(Task2Arrow)(same?mirror:-mirror);c.secondArrow=(Task2Arrow)mirror;
        c.parkToObstacle1Mm=distances[o];c.obstacleFaceGapMm=distances[g];
        float face=c.parkToObstacle1Mm+100+c.obstacleFaceGapMm;
        Task2RoutePose p={face-350,mirror*(widths[w]/2-135+80),mirror*PI/2};
        if(same)p=(Task2RoutePose){face-400,mirror*fmaxf(150,widths[w]/2-100),mirror*25*PI/180};
        p.yMm += lateral * 20.0f; p.yawRad += heading * 3.0f * PI / 180.0f;
        Task2Route route=Setup(c,!same,0);
        if(!Task2Route_ReplanObstacle2(&route,&p,widths[w],5)) {
            fprintf(stderr,"FAIL width %.0f O1 %.0f gap %.0f same %u mirror %d pose %.1f %.1f reason %s\n",
                widths[w],distances[o],distances[g],same,mirror,p.xMm,p.yMm,route.report.reason);assert(0);
        }
        Trace(&route,p,widths[w],5,false);++cases;
    }
    printf("PASS %u full return cases: all arrows, 300/500/751/800 widths, 600/1050/1500 O1 positions and gaps, +/-20 mm and +/-3 deg stops\n",cases);
    Task2RouteConfig c=task2RouteConfig;Task2Route bad=Setup(c,true,2);
    Task2RoutePose impossible={-450,250,PI};
    assert(!Task2Route_ReplanParkingReturn(&bad,&impossible,500,7));
    assert(!strcmp(bad.report.reason,"HARD_CLEARANCE_INFEASIBLE"));
    puts("PASS already-invalid hard carpark pose is never certified");
}
