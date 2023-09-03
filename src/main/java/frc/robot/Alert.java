// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.AlertsConstants.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Predicate;

/** Class for managing persistent alerts to be sent over NetworkTables. */
public class Alert {
	private static Map<String, SendableAlerts> groups = new HashMap<String, SendableAlerts>();

	private final AlertType type;
	private boolean active = false;
	private double activeStartTime = 0.0;
	private String text;

	/**
	 * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate
	 * entries will be added to NetworkTables.
	 *
	 * @param group Group identifier, also used as NetworkTables title
	 * @param text Text to be displayed when the alert is active.
	 * @param type Alert level specifying urgency.
	 */
	public Alert(String group, String text, AlertType type) {
		if (!groups.containsKey(group)) {
			groups.put(group, new SendableAlerts());
			SmartDashboard.putData(group, groups.get(group));
		}

		this.text = text;
		this.type = type;
		groups.get(group).alerts.add(this);
	}

	/**
	 * Creates a new Alert in the default group - "Alerts". If this is the first to be instantiated, the
	 * appropriate entries will be added to NetworkTables.
	 *
	 * @param text Text to be displayed when the alert is active.
	 * @param type Alert level specifying urgency.
	 */
	public Alert(String text, AlertType type) {
		this("Alerts", text, type);
	}

	/**
	 * Sets whether the alert should currently be displayed. When activated, the alert text will also be
	 * sent to the console.
	 */
	public void set(boolean active) {
		if (active && !this.active) {
			activeStartTime = Timer.getFPGATimestamp();
			switch (type) {
				case ERROR:
					DriverStation.reportError(text, false);
					break;
				case WARNING:
					DriverStation.reportWarning(text, false);
					break;
				case INFO:
					System.out.println(text);
					break;
			}
		}
		this.active = active;
	}

	/** Updates current alert text. */
	public void setText(String text) {
		if (active && !text.equals(this.text)) {
			switch (type) {
				case ERROR:
					DriverStation.reportError(text, false);
					break;
				case WARNING:
					DriverStation.reportWarning(text, false);
					break;
				case INFO:
					System.out.println(text);
					break;
			}
		}
		this.text = text;
	}

	private static class SendableAlerts implements Sendable {
		public final List<Alert> alerts = new ArrayList<>();

		public String[] getStrings(AlertType type) {
			Predicate<Alert> activeFilter = (Alert x) -> x.type == type && x.active;
			Comparator<Alert> timeSorter = (Alert a1, Alert a2) -> (int) (a2.activeStartTime - a1.activeStartTime);
			return alerts.stream().filter(activeFilter).sorted(timeSorter).map((Alert a) -> a.text).toArray(String[]::new);
		}

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.setSmartDashboardType("Alerts");
			builder.addStringArrayProperty("errors", () -> getStrings(AlertType.ERROR), null);
			builder.addStringArrayProperty("warnings", () -> getStrings(AlertType.WARNING), null);
			builder.addStringArrayProperty("infos", () -> getStrings(AlertType.INFO), null);
		}
	}
}
