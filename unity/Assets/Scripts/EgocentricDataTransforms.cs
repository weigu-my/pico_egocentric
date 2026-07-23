using System;
using UnityEngine;

public static class EgocentricDataTransforms
{
    static int ValidateRgbaImage(byte[] rgbaData, int width, int height)
    {
        if (rgbaData == null)
            throw new ArgumentNullException(nameof(rgbaData));

        int expectedLength = checked(width * height * 4);
        if (rgbaData.Length != expectedLength)
        {
            throw new ArgumentException(
                $"RGBA 缓冲区长度 {rgbaData.Length} 与期望长度 {expectedLength} 不一致。",
                nameof(rgbaData));
        }

        return expectedLength;
    }

    public static void RotateRgbaImage180InPlace(byte[] rgbaData, int width, int height)
    {
        int expectedLength = ValidateRgbaImage(rgbaData, width, height);

        int pixelCount = width * height;
        for (int leftPixel = 0, rightPixel = pixelCount - 1; leftPixel < rightPixel; leftPixel++, rightPixel--)
        {
            int leftIndex = leftPixel * 4;
            int rightIndex = rightPixel * 4;

            for (int channel = 0; channel < 4; channel++)
            {
                byte temp = rgbaData[leftIndex + channel];
                rgbaData[leftIndex + channel] = rgbaData[rightIndex + channel];
                rgbaData[rightIndex + channel] = temp;
            }
        }
    }

    public static void FlipRgbaImageHorizontallyInPlace(byte[] rgbaData, int width, int height)
    {
        ValidateRgbaImage(rgbaData, width, height);

        for (int y = 0; y < height; y++)
        {
            int rowStart = y * width * 4;
            for (int leftX = 0, rightX = width - 1; leftX < rightX; leftX++, rightX--)
            {
                int leftIndex = rowStart + leftX * 4;
                int rightIndex = rowStart + rightX * 4;

                for (int channel = 0; channel < 4; channel++)
                {
                    byte temp = rgbaData[leftIndex + channel];
                    rgbaData[leftIndex + channel] = rgbaData[rightIndex + channel];
                    rgbaData[rightIndex + channel] = temp;
                }
            }
        }
    }

    public static Vector3 ConvertRightHandedPositionToUnity(Vector3 position)
    {
        return new Vector3(position.x, position.y, -position.z);
    }

    public static Quaternion ConvertRightHandedRotationToUnity(Quaternion rotation)
    {
        return new Quaternion(rotation.x, rotation.y, -rotation.z, -rotation.w);
    }

    public static Pose ConvertRightHandedPoseToUnity(Pose pose)
    {
        return new Pose(
            ConvertRightHandedPositionToUnity(pose.position),
            ConvertRightHandedRotationToUnity(pose.rotation));
    }

    public static void ConvertRightHandedPoseToUnity(
        double x, double y, double z,
        double rx, double ry, double rz, double rw,
        out double outX, out double outY, out double outZ,
        out double outRx, out double outRy, out double outRz, out double outRw)
    {
        outX = x;
        outY = y;
        outZ = -z;
        outRx = rx;
        outRy = ry;
        outRz = -rz;
        outRw = -rw;
    }
}
