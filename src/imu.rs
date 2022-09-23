struct axis_vector {
  x_value: f64,
  y_value: f64,
  z_value: f64
}

struct quaternion {
  rotation_value: u64,
  x_vector: u64,
  y_vector: u64,
  z_vector: u64
}

struct three_by_three_rotation_matrix {
  x_value: [f64; 3],
  y_value: [f64, 3],
  z_value: [f64, 3]
}

struct four_times_four_quaternion {
  rotation_value: [u64; 4],
  x_vector: [u64; 4],
  y_vector: [u64; 4],
  z_vector: [u64; 4]
}

struct jacobian_quaternion {
  rotation_value: [u64; 3],
  x_vector: [u64; 3],
  y_vector: [u64; 3],
  z_vector: [u64; 3]
}

fn quaternion_conjugate(q: &mut quaterion) {
  q.x_vector = q.x_vector * -1;
  q.y_vector = q.y_vector * -1;
  q.z_vector  q.z_vector * -1;

  q
}


//Generates a struct using the x, y and z axis
fn axis_struct(x_axis: f64, y_axis: f64, z_axis: f64) {

    //Creates the axis_vector struct
    let rotation_matrix = three_by_three_rotation_matrix {
        x_value: x_axis,
        y_value: y_axis,
        z_value: z_axis
    };

    //returns the struct
    rotation_matrix;
}

fn gyroscope_calibration(rm: &three_by_three_rotation_matrix, acc: &f64) {

  //calibrate the gyroscope to factor in gravity and the acceleration
  let gyroscope_orientation = rm * (9.83 - acceleration);
  gyroscope_orientation
}

//Multiplys two quaterions together and makes a new, four by four quaterion
fn quaternion_multiplication(first: &quaterion, second: &quaternion) {

    let new = four_times_four_quaternion {
        rotation_value: [0, 0, 0, 0],
        x_vector: [0, 0, 0, 0],
        y_vector: [0, 0, 0, 0],
        z_vector: [0, 0, 0, 0]
}


    new.rotation_value[0] = first.rotation_value * second.rotation_value;
    new.rotation_value[1] = first.x_value * second.x_value;
    new.rotation_value[2] = first.y_value * second.y_value;
    new.rotation_value[3] = first.z_value * second.z_value;

    new.x_value[0] = -1 * (first.x_value * second.x_value);
    new.x_value[1] = first.x_value * second.rotation_value;
    new.x_value[2] = -1 * (first.x_value * second.z_value);
    new.x_value[3] = first.x_value * second.y_value;

    new.y_value[0] = -1 * (first.y_value * second.y_value);
    new.y_value[1] = first.y_value * second.z_value;
    new.y_value[2] = first.y_value *  second.rotation_value;
    new.y_value[3] = -1 * (first.y_value * second.x_value);

    new.z_value[0] = -1 * (first.z_value * second.z_value);
    new.z_value[1] = -1 * (first.z_value * second.y_value);
    new.z_value[2] = first.z_value * second.x_value;
    new.z_value[3] = first.z_value * second.rotation_value;

    new_quaternion
}

fn rotation_matrix_quaternion(q: &quaternion) {

    //Creates the axis_vector struct
    let rotation_matrix = three_by_three_rotation_matrix {
        x_value: x_axis,
        y_value: y_axis,
        z_value: z_axis
    };

  rotation_matrix.x_value[0] = 2 * (q.rotation_value) -1 + (2 * (q.x_value));
  rotation_matrix.x_value[1] = 2 * ((q.x_value * q.y_value) - (q.rotation_value * q.z_value));
  rotation_matrix.x_value[2] = 2 * ((q.x_value * q.z_value) + (q.rotation_value.rotation_value * q.y_value));

  rotation_matrix.y_value[0] = 2 * ((q.x_value * q.y_value) + (q.rotation_value * q.z_value));
  rotation_matrix.y_value[1] = (2 * q.rotation_value) - 1  + (2 * q.y_value);
  rotation_matrix.y_value[2] = 2 * (q.y_value * q.z_value - q.rotation_value * q.x_value);

  rotation_matrix.z_value[0] = 2 * (q.x_value * q.z_value);
  rotation_matrix.z_value[1] = 2 * (q.y_value * q.z_value + q.rotation_value * q.x_value);
  rotation_matrix.z_value[2] = (2 * q.rotation_value ) - 1 + (2 * q.z_value);

  rotation_matrix
}

fn accelerometer_jacobian(q: &quaternion) {

    let new_accelerometer_jacobian =  jacobian_quaternion {
        rotation_value: [0, 0, 0],
        x_vector: [0, 0, 0],
        y_vector: [0, 0, 0],
        z_vector: [0, 0, 0]
  }

  new_accelerometer_jacobian.rotation_value[0] = -2 * q.y_value;
  new_accelerometer_jacobian.rotation_value[1] = -2 * q.x_value;
  new_accelerometer_jacobian.rotation_value[2] = 0;

  new_accelerometer_jacobian.x_value[0] = 2 * q.z_value;
  new_accelerometer_jacobian.x_value[1] = 2 * q.rotation_value;
  new_accelerometer_jacobian.x_value[2] = -4 * q.x_value;

  new_accelerometer_jacobian.y_value[0] = -2 * q.rotation_value;
  new_accelerometer_jacobian.y_value[1] = 2 * q.z_value;
  new_accelerometer_jacobian.y_value[2] = -4 * q.y_value;

  new_accelerometer_jacobian.z_value[0] = 2 * q.x_value;
  new_accelerometer_jacobian.z_value[1] = 2 * q.y_value;
  new_accelerometer_jacobian.z_value[2] = 0;

  new_accelerometer_jacobian
}

fn accelerometer__function(q: &quaternion, accelerometer_vector: , magnetometer_quaternion: &quaternion) {
  struct axis_vector result;
  let result = axis_vector {
      x_value: 0,
      y_value: 0,
      z_value: 0
   };

  result.x_value = 2 * accelerometer_vector.x_value * (0.5 - (q.y_value * q.y_value) - (q.z_value * q.z_value)) + 2 * accelerometer_vector.z_value * ((q.x_value * q.z_value) - (q.rotation_value * q.y_value)) - magnetometer_quaternion.x_value;
  result.y_value = 2 * accelerometer_vector.x_value * ((q.x_value * q.y_value) - (q.rotation_value * q.z_value)) + 2 * accelerometer_vector.z_value * ((q.rotation_value * q.x_value) + (q.y_value * q.z_value)) - magnetometer_quaternion.y_value;
  result.z_value = 2 * accelerometer_vector.x_value * ((q.rotation_value * q.y_value) + (q.x_value * q.z_value)) + 2 * accelerometer_vector.z_value * (0.5 - (q.x_value * q.x_value) - (q.y_value * q.y_value)) - magnetometer_quaternion.z_value;

  result
}
