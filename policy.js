const stability_epsilon = 0.00000001;
const joint_log_softmax_temperature = tf.scalar(1.0);  // TODO:
const softmax_temperature_min = tf.scalar(0.015);


const w1 = tf.variable(tf.randomNormal([23, 64]));
const b1 = tf.variable(tf.randomNormal([64]));
const dynamic_joint_state_mask_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, center: true, scale: true})
const w2 = tf.variable(tf.randomNormal([64, 64]));
const b2 = tf.variable(tf.randomNormal([64]));
const w3 = tf.variable(tf.randomNormal([3, 4]));
const b3 = tf.variable(tf.randomNormal([4]));

function policy(dynamic_joint_description, dynamic_joint_observations, dynamic_foot_description, dynamic_foot_observations, general_state) {
    let dynamic_joint_state_mask1 = tf.add(tf.matMul(dynamic_joint_description, w1), b1);
    let dynamic_joint_state_mask2 = dynamic_joint_state_mask_layernorm1.apply(dynamic_joint_state_mask1);
    let dynamic_joint_state_mask3 = tf.elu(dynamic_joint_state_mask2);
    let dynamic_joint_state_mask4 = tf.add(tf.matMul(dynamic_joint_state_mask3, w2), b2);
    let dynamic_joint_state_mask5 = tf.tanh(dynamic_joint_state_mask4);
    let dynamic_joint_state_mask6 = tf.clipByValue(dynamic_joint_state_mask5, -1.0 + stability_epsilon, 1.0 - stability_epsilon);

    let latent_dynamic_joint_state1 = tf.add(tf.matMul(dynamic_joint_observations, w3), b3);
    let latent_dynamic_joint_state2 = tf.elu(latent_dynamic_joint_state1);

    let joint_e_x = tf.exp(tf.div(dynamic_joint_state_mask6, tf.exp(joint_log_softmax_temperature).add(softmax_temperature_min)));
    let dynamic_joint_state_mask7 = tf.div(joint_e_x, tf.sum(joint_e_x, -1, true).add(stability_epsilon));
    let dynamic_joint_state_mask8 = tf.tile(tf.expandDims(dynamic_joint_state_mask7, -1), [1, 1, 4]);
    let masked_dynamic_joint_state1 = tf.mul(dynamic_joint_state_mask8, tf.expandDims(latent_dynamic_joint_state2, -2));
    let masked_dynamic_joint_state2 = masked_dynamic_joint_state1.reshape([masked_dynamic_joint_state1.shape[0], -1]);
    let dynamic_joint_latent = tf.sum(masked_dynamic_joint_state2, 0);

    return dynamic_joint_state_mask6;
}

export { policy };