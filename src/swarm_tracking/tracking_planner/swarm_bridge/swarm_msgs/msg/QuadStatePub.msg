# QuadStatePub.msg

Header header
string child_frame_id
uint8 drone_id                        # My drone id
geometry_msgs/PoseWithCovariance pose # My pose and corresponding covariance in my world frame
float64[3] gyr                        # the unbiased angular velocity (body frame) of the body origin
float64[3] vel                        # the estimated velocity (global frame) of the body origin
ObserveTeammate[] teammate            # Teammates' states observed by me



