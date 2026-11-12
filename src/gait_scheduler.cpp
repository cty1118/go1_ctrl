// Owner: Member 3. Gait scheduler.

#include "go1_ctrl/gait_scheduler.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go1 {
namespace {

double wrap(double x){ // wrap phase into one cycle
	return x - std::floor(x); 
	}

double smooth(double x){ // smooth interpolation
	x=std::max(0.,std::min(1.,x)); // first input clamped to [0,1]
	return x*x*(3-2*x); // cubic smoothstep
	} 

int index(GaitType g){ // convert gait into array index
	int i=static_cast<int>(g);
	if(i<0 || i>3) 
		throw std::invalid_argument("invalid gait");
	return i;
	}

double cross(const Vec3& a,const Vec3& b){ // 2d cross product
	return a.x()*b.y()-a.y()*b.x(); }
	}

// gait scheduler functions (compiled defaults, replaced when config loader calls configure() )
GaitParameters GaitScheduler::defaults(GaitType g) {
  switch(g) {
    case GaitType::WALK: return {1.6, {{0,.5,.75,.25}}, {{.8,.8,.8,.8}}};
    case GaitType::TROT: return {.6, {{0,.5,.5,0}}, {{.6,.6,.6,.6}}};
    case GaitType::CANTER: return {.8, {{.6,.3,.3,0}}, {{.50,.50,.50,.50}}};
    case GaitType::GALLOP: return {.7, {{.5,.65,0,.15}}, {{.45,.45,.45,.45}}};
  }
  
  throw std::invalid_argument("invalid gait");
}

GaitScheduler::GaitScheduler(){ // constructor
  for(int i=0;i<4;++i) 
	  table_[i]=defaults(static_cast<GaitType>(i));
  
  current_= table_[index(gait_)]; // fill table_ with gait params
	}

// gait configuration 
void GaitScheduler::configure(GaitType g,const GaitParameters& p){
  const int i=index(g);
  
  if(started_)
	  throw std::logic_error("configure gait before start");
  
  // validation
  if(!std::isfinite(p.period)||p.period<.1||p.period>10) 
	  throw std::invalid_argument("period outside [0.1,10] s");
  
  for(int l=0;l<4;++l) // check all 4 legs for valid ranges
	  
    if(!std::isfinite(p.offset[l])||p.offset[l]<0||p.offset[l]>=1||!std::isfinite(p.duty[l])||p.duty[l]<=0||p.duty[l]>=1)
	  throw std::invalid_argument("offset in [0,1), duty in (0,1) required");
  
  // walk helper
  if(g==GaitType::WALK) {
    for(int a=0;a<4;++a) 
		for(int b=0;b<4;++b) if(a!=b){
			double distance=wrap(p.offset[b]+p.duty[b]-p.offset[a]-p.duty[a]);
    }
  }
  
  table_[i]=p;
}

void GaitScheduler::setTransitionTime(double s){
  if(!std::isfinite(s)||s<.1) 
	  throw std::invalid_argument("transition time >=0.1 s");
  transition_time_=s;
}

void GaitScheduler::start(GaitType g,double t) {  // start gait
  if(!std::isfinite(t)) 
	  throw std::invalid_argument("nonfinite time");
  
  gait_=requested_=g; // select gait
  current_=table_[index(g)]; // load configured params
  last_=t; // record starting controller time
  cycle_=0; // reset accumilated cycle ct
 
  pending_=false; // clear prev transition and walk state
  stage_=RUN; 
  started_=true; 
  shift_.setZero(); 
  blocked_=false;
  
  // leg starting in swing stays planted until first touchdown
  for(int i=0;i<4;++i) 
	  held_[i]=wrap(-current_.offset[i])>=current_.duty[i];
	}

void GaitScheduler::request(GaitType g) {
  index(g); // validate gait
  if(stage_!=RUN) 
	  throw std::logic_error("wait for current transition to finish");
  requested_=g; pending_=(g!=gait_);
}

bool GaitScheduler::prepareWalk(double next,double now) {
  if(!ground_walk_){ 
	shift_.setZero(); 
	return true; 
	}
	
  int leg=-1; // identify swining leg or next leg to lift
  double nearest=2; // start above max possible wrapped phase distance
  
  // swing: retain that leg's support triangle; otherwise prepare next lift.
  for(int i=0;i<4;++i) {
    double ph=wrap(cycle_-current_.offset[i]);
    if(!held_[i] && ph>=current_.duty[i]){ 
		leg=i; break; 
		}
		
    double d=wrap(current_.duty[i]-ph);
	
    if(d<nearest){
		nearest=d; leg=i; 
		}
  }
  
  if(now-feedback_.stamp<0 || now-feedback_.stamp>.1 || !feedback_.com.allFinite() || !feedback_.world_R_body.allFinite()) 
		return false;
  
  Vec3 v[3]; int n=0;
  
  for(int i=0;i<4;++i) if(i!=leg) {
    if(!feedback_.contact[i] || !feedback_.feet[i].allFinite()) return false;
    v[n++]=feedback_.feet[i];
  }
  
  Vec3 center=(v[0]+v[1]+v[2])/3.; center.z()=feedback_.com.z();
  shift_=feedback_.world_R_body.transpose()*(center-feedback_.com);
  double area=cross(v[1]-v[0],v[2]-v[0]);
  
  if(std::abs(area)<1e-6) return false;
  bool inside=true;
  
  for(int j=0;j<3;++j) {
    Vec3 edge=v[(j+1)%3]-v[j];
    double len=std::hypot(edge.x(),edge.y());
    double distance=(area>0?1:-1)*cross(edge,feedback_.com-v[j])/len;
    
	if(distance<.01) 
		inside=false; // 1 cm inward margin
  }
  
  double ph=wrap(cycle_-current_.offset[leg]);
 
 // freeze before lifting, continue publishing the target until COM is inside
  bool crossing=ph<current_.duty[leg] && ph+(next-cycle_)>=current_.duty[leg]-1e-12;
  return !crossing || inside;
}

void GaitScheduler::step(double dt,double now) { // advance scheduler with blending params
  if(stage_==BLEND) {
    double a=std::min(1.,(now-blend_start_)/transition_time_), w=smooth(a);
    current_.period=from_.period+w*(target_.period-from_.period); // interpolate period
    
	for(int i=0;i<4;++i) { // interpolate four duty factors and offsets
      current_.duty[i]=from_.duty[i]+w*(target_.duty[i]-from_.duty[i]);
      current_.offset[i]=from_.offset[i]+w*(target_.offset[i]-from_.offset[i]);
    }
	
    shift_*=1-w; // reduce stored com towards 0
    
	if(a>=1) { // blend
      gait_=requested_; // request gait is active
	  current_=target_; 
	  cycle_=0; // new cycle starts at 0
	  stage_=RUN;
      for(int i=0;i<4;++i) held_[i]=wrap(-current_.offset[i])>=current_.duty[i];
    }
	
    return;
  }
  
  double next=cycle_+dt/current_.period; // next cycle ct
  blocked_=false;
  
  // if fails, cycle_ is unchannged and controller still advances
  if(gait_==GaitType::WALK && stage_==RUN && !prepareWalk(next,now)) {
    blocked_=true; return;
  }
  
  // begin transition at cycle boundary
  if(pending_ && stage_==RUN && std::floor(next)>std::floor(cycle_)) {
    // begin bridge at a global phase boundary
    stage_=DRAIN; pending_=false; // let existing swings finish
    for(int i=0;i<4;++i) held_[i]=held_[i] || wrap(cycle_-current_.offset[i])<current_.duty[i];
  }
  
  // detecting touchdown
  for(int i=0;i<4;++i) {
    bool touchdown=std::floor(next-current_.offset[i])>std::floor(cycle_-current_.offset[i]);
    
	if(touchdown) held_[i]=(stage_==DRAIN);
  }
  
  cycle_=next; // commit phase to advance
  
  if(stage_==DRAIN && std::all_of(held_.begin(),held_.end(),[](bool b){return b;})) {
    stage_=BLEND; blend_start_=now; 
	from_=current_; 
	target_=table_[index(requested_)];
  }
}

GaitState GaitScheduler::sample() const {
  GaitState g; 
  g.period=current_.period; 
  g.com_shift=shift_;
  
  for(int i=0;i<4;++i) { // create output and assign period and com shift
    
	auto& l=g.leg[i]; l.phase=wrap(cycle_-current_.offset[i]);
    
	l.contact=stage_==BLEND || held_[i] || l.phase<current_.duty[i]; // planned contact (transition/startup holds)
    
	if(l.contact) // STANCE
		l.stance_progress=std::min(1.,l.phase/current_.duty[i]);
    
	else // SWING
		l.swing_progress=(l.phase-current_.duty[i])/(1-current_.duty[i]);
  }
  return g;
}

GaitState GaitScheduler::update(double t) {
  if(!std::isfinite(t)) 
	  throw std::invalid_argument("nonfinite time");
  if(!started_) 
	  start(gait_,t);

  // Small substeps avoid skipped events, including a blocked Walk liftoff.
  while(last_<t-1e-12){ 
	double dt=std::min(.001,t-last_); last_+=dt; step(dt,last_); }
  return sample();
}
} // namespace go1
