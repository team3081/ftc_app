package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.io.IOException;

@Disabled
public class musicthing {
    //The player handling the audio
    private static MediaPlayer mediaPlayer = null;
    //Start the wubs
    public static void start(Context context) {
        if (mediaPlayer == null) mediaPlayer = MediaPlayer.create(context, R.raw.avengers);
        mediaPlayer.seekTo(0);
        mediaPlayer.start();
    }
    //Stop the wubs
    public static void stop() {
        if (mediaPlayer != null) {
            mediaPlayer.stop();
            try { mediaPlayer.prepare(); }
            catch (IOException e) {}
        }
    }
}
