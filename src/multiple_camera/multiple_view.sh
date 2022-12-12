ROS_NAMESPACE=cam_1/usb_cam rosrun image_proc image_proc &
sleep 1
ROS_NAMESPACE=cam_2/usb_cam rosrun image_proc image_proc &
sleep 1
ROS_NAMESPACE=cam_3/usb_cam rosrun image_proc image_proc &
sleep 1
ROS_NAMESPACE=cam_4/usb_cam rosrun image_proc image_proc &
sleep 1
rqt_image_view /cam_1/usb_cam/image_rect_color &
rqt_image_view /cam_2/usb_cam/image_rect_color &
rqt_image_view /cam_3/usb_cam/image_rect_color &
rqt_image_view /cam_4/usb_cam/image_rect_color 
