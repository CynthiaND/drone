use derivative::Derivative;

#[derive(Derivative)]
#[derivative(Default)]
pub struct AxisVector {
    #[derivative(Default(value = "0.0"))]
    x_vector: f64,
    #[derivative(Default(value = "0.0"))]
    y_vector: f64,
    #[derivative(Default(value = "0.0"))]
    z_vector: f64,
}

#[derive(Derivative)]
#[derivative(Default)]
pub struct Quaternion {
    #[derivative(Default(value = "0.0"))]
    pub rotation_vector: f64,
    #[derivative(Default(value = "0.0"))]
    pub x_vector: f64,
    #[derivative(Default(value = "0.0"))]
    pub y_vector: f64,
    #[derivative(Default(value = "0.0"))]
    pub z_vector: f64,
}

#[derive(Derivative)]
#[derivative(Default)]
pub struct ThreeByThreeRotationMatrix {
    #[derivative(Default(value = "[0.0, 0.0, 0.0]"))]
    pub x_vector: [f64; 3],
    #[derivative(Default(value = "[0.0, 0.0, 0.0]"))]
    pub y_vector: [f64; 3],
    #[derivative(Default(value = "[0.0, 0.0, 0.0]"))]
    pub z_vector: [f64; 3],
}

#[derive(Derivative)]
#[derivative(Default)]
pub struct FourTimesFourQuaternion {
    #[derivative(Default(value = "[0.0, 0.0, 0.0, 0.0]"))]
    pub rotation_vector: [f64; 4],
    #[derivative(Default(value = "[0.0, 0.0, 0.0, 0.0]"))]
    pub x_vector: [f64; 4],
    #[derivative(Default(value = "[0.0, 0.0, 0.0, 0.0]"))]
    pub y_vector: [f64; 4],
    #[derivative(Default(value = "[0.0, 0.0, 0.0, 0.0]"))]
    pub z_vector: [f64; 4],
}

#[derive(Derivative)]
#[derivative(Default)]
pub struct JacobianQuaternion {
    #[derivative(Default(value = "[0.0, 0.0, 0.0]"))]
    pub rotation_vector: [f64; 3],
    #[derivative(Default(value = "[0.0, 0.0, 0.0]"))]
    pub x_vector: [f64; 3],
    #[derivative(Default(value = "[0.0, 0.0, 0.0]"))]
    pub y_vector: [f64; 3],
    #[derivative(Default(value = "[0.0, 0.0, 0.0]"))]
    pub z_vector: [f64; 3],
}

//Multiplys two quaterions together and makes a new, four by four quaterion
pub fn quat_mult(first: &Quaternion, second: &Quaternion) -> FourTimesFourQuaternion {
    let new = FourTimesFourQuaternion::default();

    new.rotation_vector[0] = first.rotation_vector * second.rotation_vector;
    new.rotation_vector[1] = first.x_vector * second.x_vector;
    new.rotation_vector[2] = first.y_vector * second.y_vector;
    new.rotation_vector[3] = first.z_vector * second.z_vector;

    new.x_vector[0] = -1.0 * (first.x_vector * second.x_vector);
    new.x_vector[1] = first.x_vector * second.rotation_vector;
    new.x_vector[2] = -1.0 * (first.x_vector * second.z_vector);
    new.x_vector[3] = first.x_vector * second.y_vector;

    new.y_vector[0] = -1.0 * (first.y_vector * second.y_vector);
    new.y_vector[1] = first.y_vector * second.z_vector;
    new.y_vector[2] = first.y_vector * second.rotation_vector;
    new.y_vector[3] = -1.0 * (first.y_vector * second.x_vector);

    new.z_vector[0] = -1.0 * (first.z_vector * second.z_vector);
    new.z_vector[1] = -1.0 * (first.z_vector * second.y_vector);
    new.z_vector[2] = first.z_vector * second.x_vector;
    new.z_vector[3] = first.z_vector * second.rotation_vector;

    new
}

pub fn quat_conj(q: &mut Quaternion) -> &mut Quaternion {
    q.x_vector = q.x_vector * -1.0;
    q.y_vector = q.y_vector * -1.0;
    q.z_vector = q.z_vector * -1.0;

    q
}

pub fn rotation_matrix_quaternion(q: &Quaternion) -> ThreeByThreeRotationMatrix {
    //Creates the axis_vector struct
    let rotation_matrix = ThreeByThreeRotationMatrix::default();

    rotation_matrix.x_vector[0] = (2.0 * (q.rotation_vector) - 1.0 + (2.0 * (q.x_vector))) as f64;
    rotation_matrix.x_vector[1] =
        (2.0 * ((q.x_vector * q.y_vector) - (q.rotation_vector * q.z_vector))) as f64;
    rotation_matrix.x_vector[2] =
        (2.0 * ((q.x_vector * q.z_vector) + (q.rotation_vector * q.y_vector))) as f64;

    rotation_matrix.y_vector[0] =
        (2.0 * ((q.x_vector * q.y_vector) + (q.rotation_vector * q.z_vector))) as f64;
    rotation_matrix.y_vector[1] = ((2.0 * q.rotation_vector) - 1.0 + (2.0 * q.y_vector)) as f64;
    rotation_matrix.y_vector[2] =
        (2.0 * (q.y_vector * q.z_vector - q.rotation_vector * q.x_vector)) as f64;

    rotation_matrix.z_vector[0] = (2.0 * (q.x_vector * q.z_vector)) as f64;
    rotation_matrix.z_vector[1] =
        (2.0 * (q.y_vector * q.z_vector + q.rotation_vector * q.x_vector)) as f64;
    rotation_matrix.z_vector[2] = ((2.0 * q.rotation_vector) - 1.0 + (2.0 * q.z_vector)) as f64;

    rotation_matrix
}
