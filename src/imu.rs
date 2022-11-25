use crate::imu::maths::*;

mod maths;

//Generates a struct using the x, y and z axis
fn axis_struct(x_axis: [f64; 3], y_axis: [f64; 3], z_axis: [f64; 3]) {
    //Creates the axis_vector struct
    let rotation_matrix = maths::ThreeByThreeRotationMatrix {
        x_vector: x_axis,
        y_vector: y_axis,
        z_vector: z_axis,
    };

    //returns the struct
    rotation_matrix;
}

fn accelerometer_jacobian(q: &Quaternion) -> JacobianQuaternion {
    let new = maths::JacobianQuaternion::default();

    new.rotation_vector[0] = -2.0 * q.y_vector;
    new.rotation_vector[1] = -2.0 * q.x_vector;
    new.rotation_vector[2] = 0.0;

    new.x_vector[0] = 2.0 * q.z_vector;
    new.x_vector[1] = 2.0 * q.rotation_vector;
    new.x_vector[2] = -4.0 * q.x_vector;

    new.y_vector[0] = -2.0 * q.rotation_vector;
    new.y_vector[1] = 2.0 * q.z_vector;
    new.y_vector[2] = -4.0 * q.y_vector;

    new.z_vector[0] = 2.0 * q.x_vector;
    new.z_vector[1] = 2.0 * q.y_vector;
    new.z_vector[2] = 0.0;

    new
}

fn accelerometer__function(
    q: &Quaternion,
    accelerometer_vector: &vector,
    magnetometer_quaternion: &Quaternion,
) {
    let result = maths::axis_vector::default();

    result.x_vector = 2.0
        * accelerometer_vector.x_vector
        * (0.5 - (q.y_vector * q.y_vector) - (q.z_vector * q.z_vector))
        + 2.0
            * accelerometer_vector.z_vector
            * ((q.x_vector * q.z_vector) - (q.rotation_vector * q.y_vector))
        - magnetometer_quaternion.x_vector;
    result.y_vector = 2.0
        * accelerometer_vector.x_vector
        * ((q.x_vector * q.y_vector) - (q.rotation_vector * q.z_vector))
        + 2.0
            * accelerometer_vector.z_vector
            * ((q.rotation_vector * q.x_vector) + (q.y_vector * q.z_vector))
        - magnetometer_quaternion.y_vector;
    result.z_vector = 2.0
        * accelerometer_vector.x_vector
        * ((q.rotation_vector * q.y_vector) + (q.x_vector * q.z_vector))
        + 2.0
            * accelerometer_vector.z_vector
            * (0.5 - (q.x_vector * q.x_vector) - (q.y_vector * q.y_vector))
        - magnetometer_quaternion.z_vector;

    result
}

fn gyroscope_calibration(rm: ThreeByThreeRotationMatrix, acc: &f64) -> ThreeByThreeRotationMatrix {
    //calibrate the gyroscope to factor in gravity and the acceleration
    let gyro = ThreeByThreeRotationMatrix::default();

    gyro.x_vector[0] = rm.x_vector[0] * (9.83 - acc);
    gyro.x_vector[1] = rm.x_vector[1] * (9.83 - acc);
    gyro.x_vector[2] = rm.x_vector[2] * (9.83 - acc);
    gyro.y_vector[0] = rm.y_vector[0] * (9.83 - acc);
    gyro.y_vector[1] = rm.y_vector[1] * (9.83 - acc);
    gyro.y_vector[2] = rm.y_vector[2] * (9.83 - acc);
    gyro.z_vector[0] = rm.z_vector[0] * (9.83 - acc);
    gyro.z_vector[1] = rm.z_vector[1] * (9.83 - acc);
    gyro.z_vector[2] = rm.z_vector[2] * (9.83 - acc);

    gyro
}
