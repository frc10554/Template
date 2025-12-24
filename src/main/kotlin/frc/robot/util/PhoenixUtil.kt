package frc.robot.util

import com.ctre.phoenix6.StatusCode
import java.util.function.Supplier

object PhoenixUtil {
    /** Attempts to run the command until no error is produced.  */
    fun tryUntilOk(maxAttempts: Int, command: Supplier<StatusCode>) {
        for (i in 0..<maxAttempts) {
            val error = command.get()
            if (error.isOK) break
        }
    }
}
