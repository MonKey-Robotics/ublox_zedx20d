# this message contains a UBX-NAV-DAHEADING (0x01 0x45) record
# Dual-antenna heading: relative positioning information in the NED frame
# between antenna 1 and antenna 2 of the module
std_msgs/Header header # Header timestamp should be acquisition time

uint8 version             # message version (0x02 for this version)
uint32 itow               # ms - GPS Time of week of the navigation epoch
int32 rel_pos_n           # mm - North component of relative position vector from antenna 1 to antenna 2
int32 rel_pos_e           # mm - East component of relative position vector from antenna 1 to antenna 2
int32 rel_pos_d           # mm - Down component of relative position vector from antenna 1 to antenna 2
int32 rel_pos_length      # mm - Length of the relative position vector
int32 rel_pos_heading     # deg scale 1e-5 - clockwise angle from True North to the projection of the
                          # antenna 1 to antenna 2 baseline, range 0 to 360 deg; adjusted by CFG_NAVSPG_DAHEADING_OFFSET
uint32 acc_n              # mm - Accuracy of relative position North component
uint32 acc_e              # mm - Accuracy of relative position East component
uint32 acc_d              # mm - Accuracy of relative position Down component
uint32 acc_length         # mm - Accuracy of length of the relative position vector
uint32 acc_heading        # deg scale 1e-5 - Accuracy of heading of the relative position vector
bool gnss_fix_ok          # A valid fix (i.e within DOP & accuracy masks)
bool diff_soln            # differential corrections were applied
bool rel_pos_valid        # relative position components, accuracy and baseline are valid
CarrSoln carr_soln        # carrier phase range solution
bool rel_pos_heading_valid # relPosHeading is valid
