#include "PhotoConsistency.h"

double calcNCC(Patch & p1, Patch & p2)
{
	return (p1.matIntensity - p1.average()).dot(p2.matIntensity - p2.average()) / (p1.stdDeri()*p2.stdDeri());
}

Mat_<Vec3d> Patch::average()
{
	return Mat_<Vec3d>();
}

double Patch::stdDeri()
{
	return 0.0;
}

Patch::Patch(vector<Vec3d> intensity)
{
	m_patchSize = intensity.size();
	m_channels = 3;
	matIntensity = Mat_<Vec3d>(m_patchSize, 1);
	for (int i = 0; i < m_patchSize; i++)
	{
		matIntensity(i) = intensity[i];
	}
}
