#include "data_generator.h"

#define WITH_NOISE 1
#define DIST 4.5

DataGenerator::DataGenerator()
{
    srand(0);
    t = 0;
    current_id = 0;;
    for (int i = 0; i < NUM_POINTS; i++)
    {
        pts[i * 3 + 0] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts[i * 3 + 1] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts[i * 3 + 2] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
    }

    //ap[0] = Vector3d( MAX_BOX, -MAX_BOX,  1);
    ap[0] = Vector3d(0, 0, 1);
    //ap[1] = Vector3d(-MAX_BOX,  MAX_BOX,  MAX_BOX);
    //ap[2] = Vector3d(-MAX_BOX, -MAX_BOX, -MAX_BOX);
    //ap[3] = Vector3d( MAX_BOX,  MAX_BOX, -MAX_BOX);
    /*
    Ric[0] <<
           0, 0, -1,
           -1, 0, 0,
           0, 1, 0;
    Tic[0] << 0.0, 0.5, 0.0;
    if (NUMBER_OF_CAMERA >= 2)
    {
        Ric[1] <<
               0, 0, -1,
               -1, 0, 0,
               0, 1, 0;
        Tic[1] << 0.0, -0.5, 0.0;
    }
    */
    //acc_cov << 1.3967e-04, 1.4357e-06, 2.1468e-06,
    //        1.4357e-06, 1.4352e-04, 5.7168e-05,
    //        2.1468e-06, 5.7168e-05, 1.5757e-04;
    //acc_cov << 1.3967e-04, 0, 0,
    //        0, 1.4352e-04, 0,
    //        0, 0, 1.5757e-04;
    acc_cov = 1e-2 * Matrix3d::Identity();
    gyr_cov = 1e-4 * Matrix3d::Identity();


    pts_cov << .1 * .1 / 3.6349576068362910e+02 / 3.6349576068362910e+02, 0,
            0, .1 * .1 / 3.6356654972681025e+02 / 3.6356654972681025e+02;



    generator = default_random_engine(0);
    distribution = normal_distribution<double>(0.0, 1);
}



void DataGenerator::update()
{
    t += 1.0 / FREQ;
}

double DataGenerator::getTime()
{
    return t;
}

Vector3d DataGenerator::getPoint(int i)
{
    return Vector3d(pts[3 * i], pts[3 * i + 1], pts[3 * i + 2]);
}

Vector3d DataGenerator::getAP(int i)
{
    return ap[i];
}

Vector3d DataGenerator::getPosition()
{
    double x, y, z;
	/*
	if(t < MAX_TIME)
	{
		x = DIST * cos(t / MAX_TIME * M_PI);
		y = DIST * sin(t / MAX_TIME * M_PI);
		z = 1;
	}
	else if (t >= MAX_TIME && t < 1.5 * MAX_TIME)
	{
		x = -DIST;
		y = 0;
		z = 1;
	}
	else
	{
		double tt = t - 0.5 * MAX_TIME;
		x = DIST * cos(tt / MAX_TIME * M_PI);
        y = DIST * sin(tt / MAX_TIME * M_PI);
        z = 1;
	}
	*/

    if (t < MAX_TIME)
    {
        x = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI);
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * 2 * 2);
        //z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * 2);
		z = 1;
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        x = MAX_BOX / 2.0 - MAX_BOX / 2.0;
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0;
        //z = MAX_BOX / 2.0 + MAX_BOX / 2.0;
		z = 1;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        x = -MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI);
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI * 2 * 2);
        //z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI * 2);
		z = 1;
    }

    return Vector3d(x, y, z);
}

Matrix3d DataGenerator::getRotation()
{
    return (AngleAxisd(30.0 / 180 * M_PI * sin(t / MAX_TIME * M_PI * 2), Vector3d::UnitX()) * AngleAxisd(40.0 / 180 * M_PI * sin(t / MAX_TIME * M_PI * 2), Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ())).toRotationMatrix();
    //return (AngleAxisd(0, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(t / MAX_TIME * M_PI, Vector3d::UnitZ())).toRotationMatrix();
}

Vector3d DataGenerator::getAngularVelocity()
{
    const double delta_t = 0.00001;
    Matrix3d rot = getRotation();
    t += delta_t;
    Matrix3d drot = (getRotation() - rot) / delta_t;
    t -= delta_t;
    Matrix3d skew = rot.inverse() * drot;
#if WITH_NOISE
    Vector3d disturb = Vector3d(distribution(generator) * sqrt(gyr_cov(0, 0)),
                                distribution(generator) * sqrt(gyr_cov(1, 1)),
                                distribution(generator) * sqrt(gyr_cov(2, 2))
                               );
    return disturb + Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
#else
    return Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
#endif
}

Vector3d DataGenerator::getVelocity()
{
    double dx, dy, dz;

    if (t < MAX_TIME)
    {

        dx = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        dy = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2);
        //dz = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2);

		/*
		dx = DIST * -sin(t / MAX_TIME *M_PI) * (1.0 / MAX_TIME * M_PI);
		dy = DIST * cos(t / MAX_TIME *M_PI) * (1.0 / MAX_TIME * M_PI);
		*/
		dz = 0;
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        dx = 0.0;
        dy = 0.0;
        dz = 0.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
		//double tt = t - 0.5 * MAX_TIME;

        dx = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        dy = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2);
        //dz = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2);

		/*
		dx = DIST * -sin(tt / MAX_TIME *M_PI) * (1.0 / MAX_TIME *   M_PI);
        dy = DIST * cos(tt / MAX_TIME *M_PI) * (1.0 / MAX_TIME *   M_PI);
		*/
		dz = 0;
    }

    return getRotation().inverse() * Vector3d(dx, dy, dz);
}

Vector3d DataGenerator::getLinearAcceleration()
{
    double ddx, ddy, ddz;
    if (t < MAX_TIME)
    {

        ddx = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2);
        //ddz = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2);

		/*
        ddx = DIST * -cos(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = DIST * -sin(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
		*/
		ddz = 0;
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        ddx = 0.0;
        ddy = 0.0;
        ddz = 0.0;
    }
    else
    {
        //double tt = t - 0.5 * MAX_TIME;
        double tt = t - 2 * MAX_TIME;

        ddx = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2) * (1.0 / MAX_TIME * M_PI * 2 * 2);
        //ddz = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2) * (1.0 / MAX_TIME * M_PI * 2);

		/*
		ddx = DIST * -cos(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = DIST * -sin(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
		*/
		ddz = 0;
    }
#if WITH_NOISE
    Vector3d disturb = Vector3d(distribution(generator) * sqrt(acc_cov(0, 0)),
                                distribution(generator) * sqrt(acc_cov(1, 1)),
                                distribution(generator) * sqrt(acc_cov(2, 2))
                               );
    return getRotation().inverse() * (disturb + Vector3d(ddx, ddy, ddz - 9.8));
#else
    return getRotation().inverse() * Vector3d(ddx, ddy, ddz - 9.8);
#endif
}

/*
vector<pair<int, Vector3d>> DataGenerator::getImage()
{
    vector<pair<int, Vector3d>> image;
    Vector3d position = getPosition();
    Matrix3d quat = getRotation();
    printf("max: %d\n", current_id);

    vector<int> ids[NUMBER_OF_CAMERA], gr_ids[NUMBER_OF_CAMERA];
    vector<Vector3d> pro_pts[NUMBER_OF_CAMERA];
    for (int k = 0; k < NUMBER_OF_CAMERA; k++)
    {
        for (int i = 0; i < NUM_POINTS; i++)
        {
            double xx = pts[i * 3 + 0] - position(0);
            double yy = pts[i * 3 + 1] - position(1);
            double zz = pts[i * 3 + 2] - position(2);
            Vector3d local_point = Ric[k].inverse() * (quat.inverse() * Vector3d(xx, yy, zz) - Tic[k]);
            xx = local_point(0);
            yy = local_point(1);
            zz = local_point(2);

            if (std::fabs(atan2(xx, zz)) <= M_PI * FOV / 2 / 180
                    && std::fabs(atan2(yy, zz)) <= M_PI * FOV / 2 / 180)
            {
                int n_id = before_feature_id[k].find(i) == before_feature_id[k].end() ?
                           -1 current_id++ : before_feature_id[k][i];
                ids[k].push_back(n_id);
                gr_ids[k].push_back(i);
                pro_pts[k].push_back(Vector3d(xx, yy, zz));
            }
        }
    }

    for (int k = 0; k < NUMBER_OF_CAMERA - 1; k++)
    {
        for (int i = 0; i < int(ids[k].size()); i++)
        {
            if (ids[k][i] != -1)
                continue;
            for (int j = 0; j < int(ids[k + 1].size()); j++)
                if (ids[k + 1][j] == -1 && gr_ids[k][i] == gr_ids[k + 1][j])
                    ids[k][i] = ids[k + 1][j] = current_id++;
        }
    }

    for (int k = 0; k < NUMBER_OF_CAMERA; k++)
    {
        for (int i = 0; i < int(ids[k].size()); i++)
        {
            if (ids[k][i] == -1)
                ids[k][i] = current_id++;
            current_feature_id[k][gr_ids[k][i]] = ids[k][i];
            image.push_back(make_pair(ids[k][i] * NUMBER_OF_CAMERA + k, pro_pts[k][i]));
        }
        std::swap(before_feature_id[k], current_feature_id[k]);
        current_feature_id[k].clear();
    }
    return image;
}
*/

vector<Vector3d> DataGenerator::getCloud()
{
    vector<Vector3d> cloud;
    for (int i = 0; i < NUM_POINTS; i++)
    {
        double xx = pts[i * 3 + 0];
        double yy = pts[i * 3 + 1];
        double zz = pts[i * 3 + 2];
        cloud.push_back(Vector3d(xx, yy, zz));
    }
    return cloud;
}
