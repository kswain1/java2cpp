import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

/*
 * After much deliberation, it was decided to put all the classes
 * required in a single file.
 * 
 * The justification is that this isnt really an object oriented program.
 * these classes exists solely to store data temporarily while the file is
 * read in, mathemetical operators are applied and output written to console
 * 
 * The output produced by this file is almost identical to the output 
 * produced by main.cpp the only (insignificant) difference been that the 
 * C++ version drops trailing zeros after the decimal point where as the
 * Java version is padded to four and five decimal places. That's because in
 * java with %g unless you sepfficy a precision, 6 digits are always produced.
 *
 * from C++
 * t: 54.604, A 1.5687 (0.141115 -1.26277 0.919948) V 0.247832 (0.0229681 -0.200947 0.143226) Px 0.000249998 Py -0.00217233 Pz 0.00153675
 * from java
 * t: 54.6040, A 1.56870 (0.141115 -1.26277 0.919948) V 0.247832 (0.0229681 -0.200947 0.143226) Px 0.000249998 Py -0.00217233 Pz 0.00153675
 *
 * The sets of values are the same though.
 */
public class Quaternion {
	/**
	 * Degrees to radians conversion factor
	 */
	static final double d2r = Math.PI / 180;
	/**
	 * Gravitational constant.
	 */
	static final double g0 = 9.81;
	
	public static void main(String[] args) {
		String filename = "../accel_mag.csv";
		if(args.length != 1) {
			System.out.println("no input file specified - assuming ../accel_mag.csv\n");
		}
		else {
			filename = args[0];
		}
		try {
			ArrayList<IMURec> records = new ArrayList<>();
			
			/*
			 * The CSV file is read in using a scanner. Processing the line
			 * after it has been read is delegated the IMURec class
			 */
			Scanner sc = new Scanner(new File(filename));
			sc.nextLine();
			while(sc.hasNext()) {
				
				IMURec rec = new IMURec(sc.nextLine());
				records.add(rec);
			}
			
			sc.close();
			/*
			 *  Quaternaion sensor -> ground need to calculate from initial ground vector
			 */
			SQ qsg = new SQ(0,0,0,1); 
			SV cur_speed = new SV(0,0,0);
			SV cur_position = new SV(0,0,0);
			
			
			int g_timeout = 100;
			int g_low_time = 100;
			
			for (int x = 1; x < records.size() - 500; x++) {
				IMURec rec = records.get(x);
				
				double dt = records.get(x).t - records.get(x-1).t;
				double gg = records.get(x+30).ax * records.get(x+30).ax + 
						    records.get(x+30).ay * records.get(x+30).ay + 
						    records.get(x+30).az * records.get(x+30).az;
				
				gg = Math.sqrt(gg);
				
				if(gg >= 5.0)
					g_low_time = 0;
				
				if(gg < 5.0)
				{
					g_low_time++;
					if(g_low_time > g_timeout)
					{
						cur_speed.x = 0;
						cur_speed.y = 0;
						cur_speed.z = 0;

						cur_position.x = 0;
						cur_position.y = 0;
						cur_position.z = 0;
					}
				}
				
				double wx = d2r * rec.wx;
				double wy = d2r * rec.wy;
				double wz = d2r * rec.wz;
				
				SQ dQsg_x = new SQ(Math.sin(0.5 * wx * dt), // x
								 0, // y 
								 0, // z
								 Math.cos(0.5 * wx * dt));
				
				SQ dQsg_y = new SQ(0, // x
						           Math.sin(0.5 * wy * dt),
						           0, // z
						           Math.cos(0.5 * wy * dt));
				
				SQ dQsg_z = new SQ(0,
								   0, 
								   Math.sin(0.5 * wz * dt),
								   Math.cos(0.5 * wz * dt));
				
				qsg = SQ.mult(qsg, dQsg_x);
				qsg = SQ.mult(qsg, dQsg_y);
				qsg = SQ.mult(qsg, dQsg_z);
				qsg.renorm();
				
				SV accel = new SV(rec.ax, rec.ay, rec.az);

				qsg.rotate(accel);

				accel.z += 1;
				
				cur_speed.x += accel.x * dt * g0;
				cur_speed.y += accel.y * dt * g0;
				cur_speed.z += accel.z * dt * g0;
				
				cur_position.x += cur_speed.x*dt;
				cur_position.y += cur_speed.y*dt;
				cur_position.z += cur_speed.z*dt;
				
				if(g_low_time < g_timeout)
				{
					System.out.printf("t: %g, A %g (%g %g %g) ", rec.t, accel.norm(), accel.x, accel.y, accel.z);
					System.out.printf("V %g (%g %g %g) ", cur_speed.norm(), cur_speed.x, cur_speed.y, cur_speed.z);
					System.out.printf("Px %g Py %g Pz %g\n", cur_position.x, cur_position.y, cur_position.z);
				}
				
			}
		} catch (FileNotFoundException e) {
			System.out.println("The file you specified could not be found.");
		}
	}
}

class SV {
	double x;
	double y;
	double z;
	
	/*
	 * Default constructor initializes all the values to 0
	 */
	SV() {
		
	}
	SV(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	/**
	 * Calculates the norm of this point.
	 * @return
	 */
	double norm() {
		return Math.sqrt(x * x + y * y + z * z);
	}

}

class SQ extends SV {
	double w;
	
	SQ() {
		super();
	}
	
	SQ(double x, double y, double z, double w) {
		super(x,y,z);
		this.w = w;
	}
	double norm() {
		return Math.sqrt(x * x + y * y + z * z + w * w);
	}
	
	void renorm() {
		/* https://github.com/e4c5/java2cpp/blob/master/quaternion.cpp#L13 */
		double r = norm();
		if(r > 0) {
			double m = 1.0 / r;
			x *= m;
			y *= m;
			z *= m;
			w *= m;
		}
	}
	
	SQ getConjugate() {
		return new SQ( -x, -y, -z, w);
	}
	
	void makeConjugate() {
		/*
		 * https://github.com/e4c5/java2cpp/blob/master/quaternion.cpp#L36
		 * does not negate w
		 */
		x = -x;
		y = -y;
		z = -z;
	}
	
	static SQ mult(SQ q1, SQ q2) {
		SQ q = new SQ();
		q.w = q1.w*q2.w - (q1.x*q2.x + q1.y*q2.y + q1.z*q2.z);
		q.x = q1.w*q2.x + q2.w*q1.x + q1.y*q2.z - q1.z*q2.y;
		q.y = q1.w*q2.y + q2.w*q1.y + q1.z*q2.x - q1.x*q2.z;
		q.z = q1.w*q2.z + q2.w*q1.z + q1.x*q2.y - q1.y*q2.x;
		return q;
	}
	
	void rotate(SV v) {
		SQ r = new SQ(v.x, v.y, v.z, 0);
		SQ qc = getConjugate();
		SQ qq = SQ.mult(r, qc);
		SQ rq = SQ.mult(this, qq);
		
		v.x = rq.x;
		v.y = rq.y;
		v.z = rq.z;
	}
	
	static SQ vMult(SQ q1, SQ q2) {
		return new SQ(q1.y * q2.z - q1.z * q2.y,
					  q1.z * q1.z - q1.x * q2.z,
					  q1.x * q2.y - q1.y * q2.x,
					  0
				);
				
	}
}

class IMURec {
	double t;
	double ax;
	double ay;
	double az;
	double wx;
	double wy;
	double wz;
	
	/**
	 * Constructs an IMURec object from a line of text.
	 * The text contains the values for each of the fields in IMURec separed
	 * by ';' characters.
	 * 
	 * @param line String
	 *       the input line of text
	 */
	public IMURec(String line) {
		String[] s = line.split(";");
		t = Double.parseDouble(s[0]);
		ax = Double.parseDouble(s[1]);
		ay = Double.parseDouble(s[2]);
		az = Double.parseDouble(s[3]);
		wx = Double.parseDouble(s[4]);
		wy = Double.parseDouble(s[5]);
		wz = Double.parseDouble(s[6]);
	}
}
