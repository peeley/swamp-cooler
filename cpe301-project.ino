enum STATE {
  DISABLED,
  IDLE,
  RUNNING,
  ERROR
};
int state = DISABLED;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  log_atmosphere();
  adjust_vent();
  check_disabled();
  // put your main code here, to run repeatedly:
  if(state == IDLE){
    if(water_level_low()){
      state = ERROR;
    }
    else if(temp_above_range()){
      toggle_fan(1);
      state = RUNNING;
    }
  }
  else if(state == RUNNING){
    if(water_level_low()){
      state = ERROR;
    }
    else if(temp_below_range()){
      toggle_fan(0);
      state = IDLE;
    }
  }
  else if(state == ERROR){
    if(!water_level_low()){
      state = IDLE;
    }
  }
}

void check_disabled(){
  
}

void idle_routine(){
  
}

void log_atmosphere(){
  // write humidity/temp
}

bool water_level_low(){
  // TODO
}

bool temp_below_range(){
  // TODO
}

bool temp_above_range(){
  // TODO
}

void toggle_fan(bool state){
  log_motor(state);
  // TODO
}

void adjust_vent(){
  // TODO
}

void log_motor(bool state){
  // TODO
}
