package org.vadere.annotation.traci.client;


import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Target(ElementType.TYPE)
@Retention(RetentionPolicy.SOURCE)
public @interface TraCIApi {
	String packageName() default "org.vadere.manager.client.traci";
	String[] imports() default {
			"org.vadere.manager.client.traci.TraCIClientApi",
			"org.vadere.manager.TraCISocket",
			"org.vadere.manager.traci.commands.TraCIGetCommand",
			"org.vadere.manager.traci.commands.TraCISetCommand",
			"org.vadere.manager.traci.respons.TraCIGetResponse",
			"org.vadere.manager.traci.writer.TraCIPacket",
			"org.vadere.manager.traci.respons.TraCIResponse",
			"java.io.IOException",
			"java.util.ArrayList"
	};
	String extendedClassName() default "TraCIClientApi";
	String name();
	Class singleAnnotation();
	Class multipleAnnotation();
	Class cmdEnum();
	Class varEnum();
}
