# Robotiq 2F-85 gripper status, published at ~50 Hz

bool   activated        # true when gACT==1 and gSTA==3 (blue light, ready)
bool   moving           # true when gOBJ==0 (finger in motion)
bool   object_detected  # true when gOBJ==1 or 2 (stopped by contact)
bool   at_position      # true when gOBJ==3 (reached requested position)

uint8  fault            # gFLT register; 0 = no fault
uint8  pos_raw          # gPO: raw position [0-255] (13=open, 230=closed)
uint8  current_raw      # gCU: raw current [0-255]

float64 position_m      # actual finger gap in metres [0.0, 0.087]
float64 current_ma      # current draw in milliamps
