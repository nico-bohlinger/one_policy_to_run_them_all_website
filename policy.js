import { w1, b1, w2, b2, w3, b3, ln_w1, ln_b1, w4, b4, ln_w2, ln_b2, w5, b5, w6, b6, w7, b7, ln_w3, ln_b3, w8, b8, w9, b9, w10, b10, ln_w4, ln_b4, w11, b11, w12, b12, ln_w5, ln_b5, w13, b13 } from './policy_weights.js';


const stability_epsilon = 0.00000001;
const joint_log_softmax_temperature = tf.scalar(-1.8753916025161743);
const foot_log_softmax_temperature = tf.scalar(-1.5039094686508179);
const softmax_temperature_min = tf.scalar(0.015);
const policy_mean_abs_clip = 10.0;

const dynamic_joint_state_mask_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, weights: [ln_w1, ln_b1], center: true, scale: true})
const dynamic_foot_state_mask_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, weights: [ln_w2, ln_b2], center: true, scale: true})
const action_latent_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, weights: [ln_w3, ln_b3], center: true, scale: true})
const action_description_latent_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, weights: [ln_w4, ln_b4], center: true, scale: true})
const policy_mean_layernorm1 = tf.layers.layerNormalization({epsilon: 1e-6, weights: [ln_w5, ln_b5], center: true, scale: true})


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