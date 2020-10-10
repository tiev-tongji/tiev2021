#include <roadrunner.h>
#include <transform.h>

void dgc_transform_print(dgc_transform_t t, char *str)
{
  int r, c;

  fprintf(stderr, "%s:\n", str);
  for(r = 0; r < 4; r++) {
    for(c = 0; c < 4; c++)
      fprintf(stderr, "%8.3f ", t[r][c]);
    fprintf(stderr, "\n");
  }
}

void dgc_transform_identity(dgc_transform_t t)
{
  int r, c;
  
  for(r = 0; r < 4; r++)
    for(c = 0; c < 4; c++)
      if(r == c)
        t[r][c] = 1;
      else
        t[r][c] = 0;
}

void dgc_transform_left_multiply(dgc_transform_t t1, dgc_transform_t t2)
{
  dgc_transform_t result;
  int i, j, k;

  for(i = 0; i < 4; i++)
    for(j = 0; j < 4; j++) {
      result[i][j] = 0;
      for(k = 0; k < 4; k++)
        result[i][j] += t2[i][k] * t1[k][j];
    }
  for(i = 0; i < 4; i++)
    for(j = 0; j < 4; j++)
      t1[i][j] = result[i][j];
}

void dgc_transform_left_multiply_nc(dgc_transform_t dest, 
				    dgc_transform_t src, 
				    dgc_transform_t left)
{
  int i, j, k;

  for(i = 0; i < 4; i++)
    for(j = 0; j < 4; j++) {
      dest[i][j] = 0;
      for(k = 0; k < 4; k++)
        dest[i][j] += left[i][k] * src[k][j];
    }
}

void dgc_transform_rotate_x(dgc_transform_t t, double theta)
{
  dgc_transform_t temp;
  double ctheta = cos(theta), stheta = sin(theta);

  dgc_transform_identity(temp);
  temp[1][1] = ctheta; 
  temp[1][2] = -stheta;
  temp[2][1] = stheta;
  temp[2][2] = ctheta;
  dgc_transform_left_multiply(t, temp);
}

void dgc_transform_rotate_y(dgc_transform_t t, double theta)
{
  dgc_transform_t temp;
  double ctheta = cos(theta), stheta = sin(theta);

  dgc_transform_identity(temp);
  temp[0][0] = ctheta; 
  temp[0][2] = stheta;
  temp[2][0] = -stheta;
  temp[2][2] = ctheta;
  dgc_transform_left_multiply(t, temp);
}

void dgc_transform_rotate_z(dgc_transform_t t, double theta)
{
  dgc_transform_t temp;
  double ctheta = cos(theta), stheta = sin(theta);

  dgc_transform_identity(temp);
  temp[0][0] = ctheta; 
  temp[0][1] = -stheta;
  temp[1][0] = stheta;
  temp[1][1] = ctheta;
  dgc_transform_left_multiply(t, temp);
}

void dgc_transform_translate(dgc_transform_t t, double x, double y, double z)
{
  t[0][3] += x;
  t[1][3] += y;
  t[2][3] += z;
}

void dgc_transform_copy(dgc_transform_t dest, dgc_transform_t src)
{
  int r, c;

  for(r = 0; r < 4; r++)
    for(c = 0; c < 4; c++)
      dest[r][c] = src[r][c];
}

void dgc_transform_get_translation(dgc_transform_t t, double *x, double *y, 
				   double *z) 
{
  *x = t[0][3];
  *y = t[1][3];
  *z = t[2][3];
}

void dgc_transform_get_rotation(dgc_transform_t t, double *x, double *y, 
				double *z) 
{
  *x = atan2(t[2][1], t[2][2]);
  *y = asin(-t[2][0]);
  *z = atan2(t[1][0], t[0][0]);                 
}

void dgc_transform_rpy(dgc_transform_t dest, dgc_transform_t src, double roll,
		       double pitch, double yaw)
{
  double sinroll = sin(roll), cosroll  = cos(roll);
  double sinpitch = sin(pitch), cospitch = cos(pitch);
  double sinyaw = sin(yaw), cosyaw = cos(yaw);
  dgc_transform_t rot;
  int i, j, k;

  /* construct rotation matrix by hand */
  rot[0][0] = cosyaw * cospitch;
  rot[0][1] = cosyaw * sinpitch * sinroll - sinyaw * cosroll;
  rot[0][2] = cosyaw * sinpitch * cosroll + sinyaw * sinroll;
  rot[0][3] = 0;
  rot[1][0] = sinyaw * cospitch; 
  rot[1][1] = sinyaw * sinpitch * sinroll + cosyaw * cosroll;
  rot[1][2] = sinyaw * sinpitch * cosroll - cosyaw * sinroll;
  rot[1][3] = 0;
  rot[2][0] = -sinpitch;
  rot[2][1] = cospitch * sinroll;
  rot[2][2] = cospitch * cosroll;
  rot[2][3] = 0;
  rot[3][0] = 0;
  rot[3][1] = 0;
  rot[3][2] = 0;
  rot[3][3] = 1;
  
  /* left multiply */
  for(i = 0; i < 4; i++)
    for(j = 0; j < 4; j++) {
      dest[i][j] = 0;
      for(k = 0; k < 4; k++)
        dest[i][j] += rot[i][k] * src[k][j];
    }
}

void dgc_transform_inverse(dgc_transform_t in, dgc_transform_t out)
{
  double temp, t1, t2, t3;

  dgc_transform_copy(out, in);

  temp = out[0][1];
  out[0][1] = out[1][0];
  out[1][0] = temp;

  temp = out[0][2];
  out[0][2] = out[2][0];
  out[2][0] = temp;

  temp = out[1][2];
  out[1][2] = out[2][1];
  out[2][1] = temp;

  t1 = 
    -out[0][0] * out[0][3]
    -out[0][1] * out[1][3]
    -out[0][2] * out[2][3];
  t2 = 
    -out[1][0] * out[0][3]
    -out[1][1] * out[1][3]
    -out[1][2] * out[2][3];
  t3 = 
    -out[2][0] * out[0][3]
    -out[2][1] * out[1][3]
    -out[2][2] * out[2][3];

  out[0][3] = t1;
  out[1][3] = t2;
  out[2][3] = t3;
}
