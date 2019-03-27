package frc.robot.util;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Random;

public class LogUtil {

	private static SimpleDateFormat DATE_FORMAT = new SimpleDateFormat("EEE', 'dd' 'MMM' 'yyyy' 'HH:mm:ss' 'Z",
			Locale.US);

	public static String getTimestamp() {
		return DATE_FORMAT.format(new Date());
	}

	public static String fromBool(boolean in) {
		return in ? "1" : "0";
	}

	public static String genSessionName() {
		// Avoids having to check if a file exists
		// Unlikely to cause a collision
		int seed = (new Random()).nextInt();
		String name = Integer.toHexString(seed).toUpperCase();
		return name;
	}
}