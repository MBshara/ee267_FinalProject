#include "OrientationMath.h"

/** TODO: see documentation in header file */
double computeAccPitch(double acc[3]) {

  int signAccY = (acc[1] >= 0) * 1 + (acc[1] < 0) * (-1);
  double pitch = -atan2(acc[2],
      signAccY * sqrt(sq(acc[0]) + sq(acc[1]))) * RAD_TO_DEG;
  return pitch;

}

/** TODO: see documentation in header file */
double computeAccRoll(double acc[3]) {

  return -atan2(-acc[0], acc[1]) * RAD_TO_DEG;

}

/** TODO: see documentation in header file */
double computeFlatlandRollGyr(double flatlandRollGyrPrev, double gyr[3], double deltaT) {

  return flatlandRollGyrPrev + deltaT * gyr[2];

}

/** TODO: see documentation in header file */
double computeFlatlandRollAcc(double acc[3]) {

  return RAD_TO_DEG * atan2(acc[0], acc[1]);

}

/** TODO: see documentation in header file */
double computeFlatlandRollComp(double flatlandRollCompPrev, double gyr[3], double flatlandRollAcc, double deltaT, double alpha) {

  return alpha * ( flatlandRollCompPrev + deltaT * gyr[2] ) +
    (1 - alpha) * flatlandRollAcc ;

}


/** TODO: see documentation in header file */
void updateQuaternionGyr(Quaternion& q, double gyr[3], double deltaT) {

  // integrate gyro
  double normW = sqrt( gyr[0]*gyr[0] + gyr[1]*gyr[1] + gyr[2]*gyr[2] ); // shouldn't matter that it's in deg/s
  Quaternion qDelta;
  if (normW >= 1e-8) {
    // really important to prevent division by zero on Teensy!
    qDelta = Quaternion().setFromAngleAxis(
      deltaT * normW, gyr[0] / normW, gyr[1] / normW, gyr[2]/normW);
  }

  //update quaternion variable
  q = Quaternion().multiply(q,qDelta).normalize();

}


/** TODO: see documentation in header file */
void updateQuaternionComp(Quaternion& q, double gyr[3], double acc[3], double deltaT, double alpha) {

  // integrate gyro
  double normW = sqrt( gyr[0]*gyr[0] + gyr[1]*gyr[1] + gyr[2]*gyr[2] ); // shouldn't matter that it's in deg/s
  Quaternion qDelta;
  if (normW >= 1e-8) {
    // really important to prevent division by zero on Teensy!
    qDelta = Quaternion().setFromAngleAxis(
      deltaT * normW, gyr[0] / normW, gyr[1] / normW, gyr[2]/normW);
  }

  Quaternion qw = Quaternion().multiply(q,qDelta).normalize();

  // get accelerometer quaternion in world
  Quaternion qa = Quaternion(0,acc[0],acc[1],acc[2]);
  qa = qa.rotate(qw);

  // compute tilt correction quaternion
  double normA = sqrt( qa.q[1]*qa.q[1] + qa.q[2]*qa.q[2] + qa.q[3]*qa.q[3] );
  double phi = RAD_TO_DEG * acos(qa.q[2]/normA);

  // tilt correction quaternion
  double normN = sqrt( qa.q[1]*qa.q[1] + qa.q[3]*qa.q[3] );
  Quaternion qt;
  if (normN >= 1e-8) { // really important to prevent division by zero on Teensy!
    qt = Quaternion().setFromAngleAxis( (1-alpha)*phi, -qa.q[3]/normN, 0.0, qa.q[1]/normN).normalize();
  }

  // update complementary filter
  q = Quaternion().multiply(qt, qw).normalize();

}



void updateQuaternionMahony(Quaternion& q,
                               double (&eInt)[3],       // persisting integration error
                               const double gyr[3],     
                               const double acc[3],     
                               double       deltaT,     
                               double       Kp  = 0.5,  // proportional gain
                               double       Ki  = 0.0)  // integral gain 
{
    double a2 = acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2];
    if (a2 < 1e-12) return; // skip bad samples
    double invA = 1.0 / sqrt(a2);
    const double ax = acc[0] * invA;
    const double ay = acc[1] * invA;
    const double az = acc[2] * invA;

    double q0 = q.q[0],  q1 = q.q[1],  q2 = q.q[2],  q3 = q.q[3];

    // predicted gravity in body frame
    const double gx =  2.0*( q1*q2 + q0*q3 );
    const double gy =  1.0 - 2.0*( q1*q1 + q3*q3 );
    const double gz =  2.0*( -q0*q1 + q2*q3 );

    // obtaining error in prediction
    const double ex = ay*gz - az*gy;
    const double ey = az*gx - ax*gz;
    const double ez = ax*gy - ay*gx;

    // computing new accumulated integral error
    if (Ki > 0.0) {
        eInt[0] += ex * deltaT;
        eInt[1] += ey * deltaT;
        eInt[2] += ez * deltaT;
    }

    // gyro feedback
    const double gxRad = (gyr[0] * M_PI/180.0)
                       + Kp*ex + Ki*eInt[0];
    const double gyRad = (gyr[1] * M_PI/180.0)
                       + Kp*ey + Ki*eInt[1];
    const double gzRad = (gyr[2] * M_PI/180.0)
                       + Kp*ez + Ki*eInt[2];

    // quaternion derivative!
    const Quaternion qDot(
        0.5*(-q1*gxRad - q2*gyRad - q3*gzRad),
        0.5*( q0*gxRad + q2*gzRad - q3*gyRad),
        0.5*( q0*gyRad - q1*gzRad + q3*gxRad),
        0.5*( q0*gzRad + q1*gyRad - q2*gxRad)
    );

    // integral and renormalize
    q.q[0] += qDot.q[0] * deltaT;
    q.q[1] += qDot.q[1] * deltaT;
    q.q[2] += qDot.q[2] * deltaT;
    q.q[3] += qDot.q[3] * deltaT;
    q.normalize();
}

void updateQuaternionMadgwick(Quaternion& q,
                                  const double gyr[3],
                                  const double acc[3],
                                  double       deltaT, 
                                  double       beta = 0.05)
{
    double aNorm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    if (aNorm < 1e-8) return; // skip bad samples
    const double ax =  acc[0] / aNorm;
    const double ay =  acc[1] / aNorm;
    const double az =  acc[2] / aNorm;

    double q0=q.q[0], q1=q.q[1], q2=q.q[2], q3=q.q[3];

    // functional errors
    double f1 = 2.0*(q0*q3 + q1*q2)- ax;
    double f2 = 2.0*(0.5 - q1*q1- q3*q3) - ay;
    double f3 = 2.0*(-q0*q1 + q2*q3) - az;

    // J.T @ f computation essentially step size
    // 2q4, 2q3, 2q2, 2q1
    // 0, -4q2, 0, -4q4
    // -2q2, -2q1, 2q4, 2q3
    // double g0 =  2.0*(  q3*f1 + q0*f2 - q1*f3 );
    // double g1 =  2.0*(  q2*f1 - q1*f2 - q0*f3 );
    // double g2 =  2.0*(  q1*f1 + q2*f2 + q3*f3 );
    // double g3 =  2.0*(  q0*f1 - q3*f2 + q2*f3 );
    double g0 =  2.0*(  -q1*f3 + q3*f1 );
    double g1 =  2.0*(  -q0*f3 - 2*q1*f2 + q2*f1);
    double g2 =  2.0*(  q3*f3 + q1*f1 );
    double g3 =  2.0*(  q0*f1 - 2*q3*f2 + q2*f3 );

    // normalization
    double gNorm = sqrt(g0*g0 + g1*g1 + g2*g2 + g3*g3);
    if (gNorm > 1e-9) { g0/=gNorm; g1/=gNorm; g2/=gNorm; g3/=gNorm; }

    const double gxRad = gyr[0] * (M_PI/180.0);
    const double gyRad = gyr[1] * (M_PI/180.0);
    const double gzRad = gyr[2] * (M_PI/180.0);

    // quaternion derivative, with update!
    Quaternion qDot(
        0.5*(-q1*gxRad - q2*gyRad - q3*gzRad)  -  beta*g0,
        0.5*( q0*gxRad + q2*gzRad - q3*gyRad)  -  beta*g1,
        0.5*( q0*gyRad - q1*gzRad + q3*gxRad)  -  beta*g2,
        0.5*( q0*gzRad + q1*gyRad - q2*gxRad)  -  beta*g3
    );

    // integral and renormalize
    q.q[0] += qDot.q[0] * deltaT;
    q.q[1] += qDot.q[1] * deltaT;
    q.q[2] += qDot.q[2] * deltaT;
    q.q[3] += qDot.q[3] * deltaT;
    q.normalize();
}

/**
 Extended Kalman Filter for orientation tracking using accelerometer and gyroscope
 
 State vector: quaternion q = [q0, q1, q2, q3]
 Process model: quaternion integration of gyroscope
 Measurement model: accelerometer measures gravity direction
 */
void updateQuaternionEKF(Quaternion& q, 
                        double (&P)[4][4], // covariance
                        const double gyr[3], 
                        const double acc[3], 
                        double deltaT,
                        double Qnoise_base = 0.01,  // process noise
                        double Rnoise_base = 0.1)    // measurement noise
{
    const double gxRad = gyr[0] * (M_PI/180.0);
    const double gyRad = gyr[1] * (M_PI/180.0);
    const double gzRad = gyr[2] * (M_PI/180.0);
    double gyroMagnitude = sqrt(gxRad*gxRad + gyRad*gyRad + gzRad*gzRad);
    
    // Adaptive noise parameters
    double Qnoise, Rnoise;
    
    if (gyroMagnitude > 1.2) { // Very fast (>69 deg/s)
        Qnoise = Qnoise_base * 20; // 0.02
        Rnoise = Rnoise_base * 3; // 0.3
    } else if (gyroMagnitude > 0.5) { // Moderate (>29 deg/s)
        Qnoise = Qnoise_base * 5; // 0.005
        Rnoise = Rnoise_base * 1.5; // 0.15
    } else { // Slow movement
        Qnoise = Qnoise_base; // 0.001 (your smooth setting)
        Rnoise = Rnoise_base;  // 0.1
    }
    
    double q0 = q.q[0], q1 = q.q[1], q2 = q.q[2], q3 = q.q[3];
    
    // Quaternion derivative
    double qDot[4] = {
        0.5 * (-q1*gxRad - q2*gyRad - q3*gzRad),
        0.5 * ( q0*gxRad + q2*gzRad - q3*gyRad),
        0.5 * ( q0*gyRad - q1*gzRad + q3*gxRad),
        0.5 * ( q0*gzRad + q1*gyRad - q2*gxRad)
    };
    
    // Integrate to get predicted quaternion
    double qPred[4] = {
        q0 + qDot[0] * deltaT,
        q1 + qDot[1] * deltaT,
        q2 + qDot[2] * deltaT,
        q3 + qDot[3] * deltaT
    };
    
    // Normalize predicted quaternion
    double qNorm = sqrt(qPred[0]*qPred[0] + qPred[1]*qPred[1] + 
                       qPred[2]*qPred[2] + qPred[3]*qPred[3]);
    if (qNorm > 1e-8) {
        qPred[0] /= qNorm; qPred[1] /= qNorm; 
        qPred[2] /= qNorm; qPred[3] /= qNorm;
    }
    
    // Compute state transition Jacobian F
    double F[4][4] = {
        {1.0,              -0.5*gxRad*deltaT, -0.5*gyRad*deltaT, -0.5*gzRad*deltaT},
        {0.5*gxRad*deltaT,  1.0,               0.5*gzRad*deltaT, -0.5*gyRad*deltaT},
        {0.5*gyRad*deltaT, -0.5*gzRad*deltaT,  1.0,               0.5*gxRad*deltaT},
        {0.5*gzRad*deltaT,  0.5*gyRad*deltaT, -0.5*gxRad*deltaT,  1.0}
    };
    
    // Predict covariance: P = F * P * F' + Q
    double PTemp[4][4] = {0};
    
    // Compute F * P
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                PTemp[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    
    // Compute (F * P) * F' and add process noise
    double PPred[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            PPred[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                PPred[i][j] += PTemp[i][k] * F[j][k];
            }
            // Add process noise on diagonal
            if (i == j) PPred[i][j] += Qnoise;
        }
    }
    
    double aNorm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    if (aNorm < 1e-8) { // skip bad samples
        q.q[0] = qPred[0]; q.q[1] = qPred[1];
        q.q[2] = qPred[2]; q.q[3] = qPred[3];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P[i][j] = PPred[i][j];
            }
        }
        return;
    }
    
    const double ax = acc[0] / aNorm;
    const double ay = acc[1] / aNorm;
    const double az = acc[2] / aNorm;
    
    q0 = qPred[0]; q1 = qPred[1]; q2 = qPred[2]; q3 = qPred[3];
    
    double hx = 2.0 * (q0*q3 + q1*q2);
    double hy = 1.0 - 2.0*(q1*q1 + q3*q3);
    double hz = 2.0 * (-q0*q1 + q2*q3);
    
    // innovation residual
    double y[3] = {ax - hx, ay - hy, az - hz};
    
    // H jacobian
    double H[3][4] = {
        { 2*q3,  2*q2,  2*q1,  2*q0},
        { 0, -4*q1,  0, -4*q3},  
        {-2*q1, -2*q0,  2*q3,  2*q2}
    };
    
    // Compute innovation covariance S = H * P * H' + R
    double HP[3][4] = {0};
    
    // Compute H * P
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                HP[i][j] += H[i][k] * PPred[k][j];
            }
        }
    }
    
    // Compute S = (H * P) * H' + R
    double S[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 4; k++) {
                S[i][j] += HP[i][k] * H[j][k];
            }
            // add measurement noise on diagonal
            if (i == j) S[i][j] += Rnoise;
        }
    }
    
    // Kalman gain K = P * H' * S^(-1)
    // First compute S inverse (3x3 matrix)
    double detS = S[0][0] * (S[1][1]*S[2][2] - S[1][2]*S[2][1]) -
                  S[0][1] * (S[1][0]*S[2][2] - S[1][2]*S[2][0]) +
                  S[0][2] * (S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    
    if (fabs(detS) < 1e-8) {
        // singular matrix, skip update
        q.q[0] = qPred[0]; q.q[1] = qPred[1];
        q.q[2] = qPred[2]; q.q[3] = qPred[3];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P[i][j] = PPred[i][j];
            }
        }
        return;
    }
    
    double SInv[3][3];
    SInv[0][0] = (S[1][1]*S[2][2] - S[1][2]*S[2][1]) / detS;
    SInv[0][1] = (S[0][2]*S[2][1] - S[0][1]*S[2][2]) / detS;
    SInv[0][2] = (S[0][1]*S[1][2] - S[0][2]*S[1][1]) / detS;
    SInv[1][0] = (S[1][2]*S[2][0] - S[1][0]*S[2][2]) / detS;
    SInv[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0]) / detS;
    SInv[1][2] = (S[0][2]*S[1][0] - S[0][0]*S[1][2]) / detS;
    SInv[2][0] = (S[1][0]*S[2][1] - S[1][1]*S[2][0]) / detS;
    SInv[2][1] = (S[0][1]*S[2][0] - S[0][0]*S[2][1]) / detS;
    SInv[2][2] = (S[0][0]*S[1][1] - S[0][1]*S[1][0]) / detS;
    
    double PHt[4][3] = {0};
    
    // Compute P * H'
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 4; k++) {
                PHt[i][j] += PPred[i][k] * H[j][k];
            }
        }
    }
    
    // Compute K = (P * H') * S^(-1)
    double K[4][3] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                K[i][j] += PHt[i][k] * SInv[k][j];
            }
        }
    }
    
    // update_state
    double qUpdate[4] = {qPred[0], qPred[1], qPred[2], qPred[3]};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            qUpdate[i] += K[i][j] * y[j];
        }
    }
    
    // normalized quaternion
    qNorm = sqrt(qUpdate[0]*qUpdate[0] + qUpdate[1]*qUpdate[1] + 
                 qUpdate[2]*qUpdate[2] + qUpdate[3]*qUpdate[3]);
    if (qNorm > 1e-8) {
        q.q[0] = qUpdate[0] / qNorm;
        q.q[1] = qUpdate[1] / qNorm;
        q.q[2] = qUpdate[2] / qNorm;
        q.q[3] = qUpdate[3] / qNorm;
    }
    
    // update covariance: P = (I - K * H) * P
    double KH[4][4] = {0};
    
    // Compute K * H
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 3; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }
    
    // Compute (I - K * H) * P
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double sum = 0;
            for (int k = 0; k < 4; k++) {
                double IKH = (i == k ? 1.0 : 0.0) - KH[i][k];
                sum += IKH * PPred[k][j];
            }
            P[i][j] = sum;
        }
    }
}


void initializeEKFCovariance(double (&P)[4][4], double initVar = 0.1) {
    // Initialize as diagonal matrix
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = (i == j) ? initVar : 0.0;
        }
    }
}