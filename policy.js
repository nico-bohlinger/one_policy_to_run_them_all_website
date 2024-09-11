const stability_epsilon = 0.00000001;
const joint_log_softmax_temperature = tf.scalar(1.0);  // TODO:
const foot_log_softmax_temperature = tf.scalar(1.0);  // TODO:
const softmax_temperature_min = tf.scalar(0.015);
const policy_mean_abs_clip = 10.0;


const w1 = tf.variable(tf.randomNormal([23, 64]));
const b1 = tf.variable(tf.randomNormal([64]));
const dynamic_joint_state_mask_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, center: true, scale: true})
const w2 = tf.variable(tf.randomNormal([64, 64]));
const b2 = tf.variable(tf.randomNormal([64]));
const w3 = tf.variable(tf.randomNormal([3, 4]));
const b3 = tf.variable(tf.randomNormal([4]));

const w4 = tf.variable(tf.randomNormal([10, 64]));
const b4 = tf.variable(tf.randomNormal([64]));
const dynamic_foot_state_mask_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, center: true, scale: true})
const w5 = tf.variable(tf.randomNormal([64, 64]));
const b5 = tf.variable(tf.randomNormal([64]));
const w6 = tf.variable(tf.randomNormal([2, 4]));
const b6 = tf.variable(tf.randomNormal([4]));

const w7 = tf.variable(tf.randomNormal([532, 512]));
const b7 = tf.variable(tf.randomNormal([512]));
const action_latent_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, center: true, scale: true})
const w8 = tf.variable(tf.randomNormal([512, 256]));
const b8 = tf.variable(tf.randomNormal([256]));
const w9 = tf.variable(tf.randomNormal([256, 128]));
const b9 = tf.variable(tf.randomNormal([128]));

const w10 = tf.variable(tf.randomNormal([23, 128]));
const b10 = tf.variable(tf.randomNormal([128]));
const action_description_latent_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, center: true, scale: true})
const w11 = tf.variable(tf.randomNormal([128, 128]));
const b11 = tf.variable(tf.randomNormal([128]));

const w12 = tf.variable(tf.randomNormal([260, 128]));
const b12 = tf.variable(tf.randomNormal([128]));
const policy_mean_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, center: true, scale: true})
const w13 = tf.variable(tf.randomNormal([128, 1]));
const b13 = tf.variable(tf.randomNormal([1]));


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

    let dynamic_foot_state_mask1 = tf.add(tf.matMul(dynamic_foot_description, w4), b4);
    let dynamic_foot_state_mask2 = dynamic_foot_state_mask_layernorm1.apply(dynamic_foot_state_mask1);
    let dynamic_foot_state_mask3 = tf.elu(dynamic_foot_state_mask2);
    let dynamic_foot_state_mask4 = tf.add(tf.matMul(dynamic_foot_state_mask3, w5), b5);
    let dynamic_foot_state_mask5 = tf.tanh(dynamic_foot_state_mask4);
    let dynamic_foot_state_mask6 = tf.clipByValue(dynamic_foot_state_mask5, -1.0 + stability_epsilon, 1.0 - stability_epsilon);
    let latent_dynamic_foot_state1 = tf.add(tf.matMul(dynamic_foot_observations, w6), b6);
    let latent_dynamic_foot_state2 = tf.elu(latent_dynamic_foot_state1);
    let foot_e_x = tf.exp(tf.div(dynamic_foot_state_mask6, tf.exp(foot_log_softmax_temperature).add(softmax_temperature_min)));
    let dynamic_foot_state_mask7 = tf.div(foot_e_x, tf.sum(foot_e_x, -1, true).add(stability_epsilon));
    let dynamic_foot_state_mask8 = tf.tile(tf.expandDims(dynamic_foot_state_mask7, -1), [1, 1, 4]);
    let masked_dynamic_foot_state1 = tf.mul(dynamic_foot_state_mask8, tf.expandDims(latent_dynamic_foot_state2, -2));
    let masked_dynamic_foot_state2 = masked_dynamic_foot_state1.reshape([masked_dynamic_foot_state1.shape[0], -1]);
    let dynamic_foot_latent = tf.sum(masked_dynamic_foot_state2, 0);

    let combined_input = tf.expandDims(tf.concat([dynamic_joint_latent, dynamic_foot_latent, general_state], -1), 0);

    let action_latent1 = tf.add(tf.matMul(combined_input, w7), b7);
    let action_latent2 = action_latent_layernorm1.apply(action_latent1);
    let action_latent3 = tf.elu(action_latent2);
    let action_latent4 = tf.add(tf.matMul(action_latent3, w8), b8);
    let action_latent5 = tf.elu(action_latent4);
    let action_latent6 = tf.add(tf.matMul(action_latent5, w9), b9);

    let action_description_latent1 = tf.add(tf.matMul(dynamic_joint_description, w10), b10);
    let action_description_latent2 = action_description_latent_layernorm1.apply(action_description_latent1);
    let action_description_latent3 = tf.elu(action_description_latent2);
    let action_description_latent4 = tf.add(tf.matMul(action_description_latent3, w11), b11);

    let action_latent_7 = tf.tile(action_latent6, [dynamic_joint_description.shape[0], 1]);
    let combined_action_latent = tf.concat([action_latent_7, latent_dynamic_joint_state2, action_description_latent4], -1);
    let policy_mean1 = tf.add(tf.matMul(combined_action_latent, w12), b12);
    let policy_mean2 = policy_mean_layernorm1.apply(policy_mean1);
    let policy_mean3 = tf.elu(policy_mean2);
    let policy_mean4 = tf.add(tf.matMul(policy_mean3, w13), b13);
    let policy_mean5 = tf.clipByValue(policy_mean4, -policy_mean_abs_clip, policy_mean_abs_clip);

    return policy_mean5;
}

export { policy };