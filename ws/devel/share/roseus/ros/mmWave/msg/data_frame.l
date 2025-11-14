;; Auto-generated. Do not edit!


(when (boundp 'mmWave::data_frame)
  (if (not (find-package "MMWAVE"))
    (make-package "MMWAVE"))
  (shadow 'data_frame (find-package "MMWAVE")))
(unless (find-package "MMWAVE::DATA_FRAME")
  (make-package "MMWAVE::DATA_FRAME"))

(in-package "ROS")
;;//! \htmlinclude data_frame.msg.html


(defclass mmWave::data_frame
  :super ros::object
  :slots (_data ))

(defmethod mmWave::data_frame
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int16[] _data
    (* 2    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-word (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get mmWave::data_frame :md5sum-) "8560fbebb34fa1b9472337b5c3d38fda")
(setf (get mmWave::data_frame :datatype-) "mmWave/data_frame")
(setf (get mmWave::data_frame :definition-)
      "int16[] data
")



(provide :mmWave/data_frame "8560fbebb34fa1b9472337b5c3d38fda")


