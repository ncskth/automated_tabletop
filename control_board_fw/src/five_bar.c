#include <math.h>

/*
              //  x,y
             // \\
         r2 //   \\  r3
           //     \\
          //       \\
         //         \\
        //a2         \\a3
        \\           //
      r1 \\   r5    // r4
          \\a1-----//a4
*/

int five_bar_angle_to_pos(float r1, float r2, float r3, float r4, float r5, float theta1, float theta2, float *p_x, float *p_y) {
    float P1_x = -r5 / 2.0, P1_y = 0.0;
    float P2_x = r5 / 2.0, P2_y = 0.0;

    // Calculate positions of the outer joints of r1 and r2
    float J1_x = P1_x + r1 * cos(theta1);
    float J1_y = P1_y + r1 * sin(theta1);
    float J2_x = P2_x + r2 * cos(theta2);
    float J2_y = P2_y + r2 * sin(theta2);

    // Calculate distance between J1 and J2
    float d = sqrt((J2_x - J1_x) * (J2_x - J1_x) + (J2_y - J1_y) * (J2_y - J1_y));

    if (d > r3 + r4) {
        return -1; // Unreachable
    }

    // Use the law of cosines to find the angle between r3 and the line connecting J1 and J2
    float angle_r3 = acos((r3 * r3 + d * d - r4 * r4) / (2 * r3 * d));

    // Find the angle of the line connecting J1 and J2
    float angle_line = atan2(J2_y - J1_y, J2_x - J1_x);

    // Calculate the angle to reach the end-effector position
    float angle_to_effector = angle_line + angle_r3;

    // Calculate position of the end-effector
    *p_x = J1_x + r3 * cos(angle_to_effector);
    *p_y = J1_y + r3 * sin(angle_to_effector);
    return 0;
}

int five_bar_pos_to_angle(float r1, float r2, float r3, float r4, float r5, float p_x, float p_y, float *theta1, float *theta2) {
    p_x += r5 / 2;
    float A1, B1, C1, A2, B2, C2;
    float discriminant1, discriminant2;
    float t_11, t_22;

    // First Loop
    A1 = p_x*p_x + p_y*p_y + r1*r1 + 2*r1*p_x - r2*r2;
    B1 = -4*r1*p_y;
    C1 = r1*r1 - 2*r1*p_x + p_x*p_x + p_y*p_y - r2*r2;
    discriminant1 = B1*B1 - 4*A1*C1;

    if (discriminant1 < 0) {
        return -1; // Error: No real solution exists
    }

    t_11 = (-B1 + sqrt(discriminant1)) / (2*A1);
    *theta1 = atan2(2*t_11, 1 - t_11*t_11);

    // Second Loop
    A2 = p_x*p_x + p_y*p_y + r5*r5 + r4*r4 - 2*p_x*r5 - r3*r3 + 2*p_x*r4 - 2*r4*r5;
    B2 = -4*p_y*r4;
    C2 = p_x*p_x + p_y*p_y + r5*r5 + r4*r4 - 2*p_x*r5 - r3*r3 + 2*r4*r5 - 2*p_x*r4;
    discriminant2 = B2*B2 - 4*A2*C2;

    if (discriminant2 < 0) {
        return -1; // Error: No real solution exists
    }

    t_22 = (-B2 - sqrt(discriminant2)) / (2*A2);
    *theta2 = atan2(2*t_22, 1 - t_22*t_22);

    return 0; // Success
}