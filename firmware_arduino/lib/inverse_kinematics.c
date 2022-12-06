#include <inverse_kinematics.h>

float get_phi(float x, float y) {
    return atan2(y,x);
}

void v_get_phi(float x, float y, float *phi) {
    *phi = atan2(y,x);
}


float get_theta(float x, float y) {
    return atan(sqrt(x * x + y * y) / Z_HEIGHT);
}

void v_get_theta(float x, float y, float *theta) {
    *theta = atan(sqrt(x * x + y * y) / Z_HEIGHT);
}


void get_angles(float x, float y, float *phi, float *theta) {
	*phi = get_phi(x, y);
	*theta = get_theta(x, y);
}

void get_xyz(float phi, float theta, float *x, float *y, float *z) {
	*x = sin(theta) * sin(phi) * L3;
	*y = sin(theta) * cos(phi) * L3;
	*z = cos(theta) * L3;
}

void get_z(float x, float y, float z, float *x_final, float *z_final) {
	float AB = sqrt(pow(x + L2,2) + y * y + z * z);
	float q = (L1 * L1 + AB * AB - L2 * L2) * AB * L1 / ((2 * L1 * AB) * (x + L2));
	float k = -z / (x + L2);

	float D = pow(2 * k * q, 2) - 4 * (k * k + 1) * (q * q - L1 * L1);

	float z_d1 = (-2 * k * q - sqrt(D)) / (2 * (k * k + 1));
	float z_d2 = (-2 * k * q + sqrt(D)) / (2 * (k * k + 1));
	*z_final = z_d1 >= z_d2 ? z_d1 : z_d2;
	*x_final = sqrt(L1 * L1 - pow(*z_final, 2));

	float test = sqrt(pow(x+L2 - *x_final, 2) + y * y + pow(*z_final-z, 2))*100;
	
	if (round(test)/100 != L2){
	    *x_final *=- 1;
	}
	
}

float get_final_angle(float x, float z) {
	return acos(x / L1);
}

float inverse_kinematics(float x, float y, int flip) {
	float phi;
	float theta;

	float x_A;
	float y_A;
	float z_A;

	float x_final;
	float z_final;

	get_angles(x, y, &phi, &theta);
	get_xyz(phi, theta, &x_A, &y_A, &z_A);

	if (flip == 1) {
		float temp = y_A;
		y_A = x_A;
		x_A = temp;
	}

	get_z(x_A, y_A, z_A, &x_final, &z_final);

	return get_final_angle(x_final, z_final) - M_PI_2;
}

float inverse_kinematics_angle(float phi, float theta, int flip) {
	float x_A;
	float y_A;
	float z_A;

	float x_final;
	float z_final;

	get_xyz(phi, theta, &x_A, &y_A, &z_A);

	if (flip == 1) {
		float temp = y_A;
		y_A = x_A;
		x_A = temp;
	}

	get_z(x_A, y_A, z_A, &x_final, &z_final);

	return get_final_angle(x_final, z_final) - M_PI_2;
}

void v_inverse_kinematics(float x, float y, float *angle, int flip) {
	float phi, theta;

	float x_A, y_A, z_A;

	float x_final, z_final;

	get_angles(x, y, &phi, &theta);
	get_xyz(phi, theta, &x_A, &y_A, &z_A);

	if (flip == 1) {
		float temp = y_A;
		y_A = x_A;
		x_A = temp;
	}

	get_z(x_A, y_A, z_A, &x_final, &z_final);

	*angle = get_final_angle(x_final, z_final) - M_PI_2;
}


void v_inverse_kinematics_angle(float phi, float theta, float *angle, int flip) {
	float x_A;
	float y_A;
	float z_A;

	float x_final;
	float z_final;

	get_xyz(phi, theta, &x_A, &y_A, &z_A);

	if (flip == 1) {
		float temp = y_A;
		y_A = x_A;
		x_A = temp;
	}

	get_z(x_A, y_A, z_A, &x_final, &z_final);

	*angle = get_final_angle(x_final, z_final) - M_PI_2;
}
