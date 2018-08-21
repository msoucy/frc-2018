package frc.team166.chopshoplib;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {

    public void log() {
        final String branch = getResource("branch.txt");
        SmartDashboard.putString("branch", branch);

        final String commit = getResource("commit.txt");
        SmartDashboard.putString("commit", commit);

        final String changes = getResource("changes.txt");
        SmartDashboard.putString("changes", changes);

        final String buildtime = getResource("buildtime.txt");
        SmartDashboard.putString("buildtime", buildtime);
    }

    private String getResource(final String path) {
        String resource;
        try (InputStream stream = getClass().getResourceAsStream("/" + path);
                InputStreamReader reader = new InputStreamReader(stream, StandardCharsets.UTF_8);
                BufferedReader bufferedReader = new BufferedReader(reader)) {
            resource = bufferedReader.lines()
                    .collect(Collectors.joining("\n"));
        } catch (IOException e) {
            resource = "";
        }
        return resource;
    }

}
