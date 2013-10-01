package com.sjsu.physics.utils;

public class Quaternion 
{
    private final float data[]; 

    public Quaternion ()
    {
    	data = new float[4];
    	data[0] = 1;
    	data[1] = 0;
    	data[2] = 0;
    	data[3] = 0;
    }
    // create a new object with the given components
    public Quaternion(float r, float i, float j, float k) 
    {
    	data = new float[4];
        data[0] = r;
        data[1] = i;
        data[2] = j;
        data[3] = k;
    }
    
    public float r()
    {
    	return data[0];
    }
    
    public float i()
    {
    	return data[1];
    }
    
    public float j()
    {
    	return data[2];
    }
    
    public float k()
    {
    	return data[3];
    }

    // return a string representation of the invoking object
    public String toString() 
    {
        return data[0] + " + " + data[1] + "i + " + data[2] + "j + " + data[3] + "k";
    }

    // return the quaternion norm
    public float norm() 
    {
        return (float) Math.sqrt(data[0]*data[0] + data[1]*data[1] +data[2]*data[2] + data[3]*data[3]);
    }

    // return the quaternion conjugate
    public Quaternion conjugate() 
    {
        return new Quaternion(data[0], -data[1], -data[2], -data[3]);
    }

    // return a new Quaternion whose value is (this + b)
    public Quaternion plus(Quaternion b)
    {
        Quaternion a = this;
        return new Quaternion(a.data[0]+b.data[0], a.data[1]+b.data[1], a.data[2]+b.data[2], a.data[3]+b.data[3]);
    }


    // return a new Quaternion whose value is (this * b)
    public Quaternion times(Quaternion b) 
    {
        Quaternion a = this;
        float y0 = a.data[0]*b.data[0] - a.data[1]*b.data[1] - a.data[2]*b.data[2] - a.data[3]*b.data[3];
        float y1 = a.data[0]*b.data[1] + a.data[1]*b.data[0] + a.data[2]*b.data[3] - a.data[3]*b.data[2];
        float y2 = a.data[0]*b.data[2] - a.data[1]*b.data[3] + a.data[2]*b.data[0] + a.data[3]*b.data[1];
        float y3 = a.data[0]*b.data[3] + a.data[1]*b.data[2] - a.data[2]*b.data[1] + a.data[3]*b.data[0];
        return new Quaternion(y0, y1, y2, y3);
    }

    // return a new Quaternion whose value is the inverse of this
    public Quaternion inverse() 
    {
        float d = data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3];
        return new Quaternion(data[0]/d, -data[1]/d, -data[2]/d, -data[3]/d);
    }
    
    public Quaternion normalize()
    {
    	float d = data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3];
    	
    	// check for zero length, and use no-rotation if true
    	if (d == 0)
    	{
    		return new Quaternion(1, 0, 0, 0);
    	}
    	
    	d = 1.0f / (float) Math.sqrt(d);
    	return new Quaternion(data[0] * d, data[1] * d, data[2] * d, data[3] * d);
    }

    // return a + this
    public Quaternion addScaledVector(Vector2 vector, float scale)
    {
        Quaternion q = new Quaternion(0,
            vector.x * scale,
            vector.y * scale,
            vector.y * scale);
        q = q.times(this);
        
        float r = this.r() + q.r() * ((float)0.5);
        float i = this.i() + q.i() * ((float)0.5);
        float j = this.j() + q.j() * ((float)0.5);
        float k = this.k() + q.k() * ((float)0.5);
        
        return new Quaternion(r, i, j, k);
    }

    // return a / b
    public Quaternion divides(Quaternion b) 
    {
         Quaternion a = this;
        return a.inverse().times(b);
    }
}
