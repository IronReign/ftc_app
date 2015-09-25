package org.swerverobotics.library.internal;

import android.app.Application;
import android.content.Context;
import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.*;
import org.swerverobotics.library.interfaces.*;
import java.io.*;
import java.lang.reflect.*;
import java.lang.annotation.*;
import java.util.*;

import dalvik.system.DexFile;

/**
 * Call {@linkplain AnnotatedOpModeRegistrar#register(OpModeManager)} from FtcOpModeRegister.register()
 * in order to automatically register OpMode classes that you have annotated with @Autonomous
 * or @TeleOp.
 *
 * @see Autonomous
 * @see TeleOp
 */
public class AnnotatedOpModeRegistrar
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    LinkedList<String>  partialClassNamesToIgnore;

    final OpModeManager                opModeManager;
    final Context                      context;
    final DexFile                      dexFile;

    final HashMap<String, LinkedList<Class>> pairedOpModes;
    final HashSet<Class>                     classesSeen;
    final HashMap<Class, String>             classNameOverrides;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     * Call this method from FtcOpModeRegister.register() in order to register OpModes which
     * have been annotated as {@linkplain Autonomous} or {@linkplain TeleOp} OpModes.
     *
     * @param manager   the manager used to carry out the actual registration
     */
    public static void register(final OpModeManager manager)
        {
        AnnotatedOpModeRegistrar registrar = null;
        try {
            registrar = new AnnotatedOpModeRegistrar(manager);
            }
        catch (ClassNotFoundException | NoSuchMethodException | InvocationTargetException | IllegalAccessException | IOException e)
            {
            registrar = null;
            }

        if (registrar != null)
            registrar.doRegistration();
        }

    private AnnotatedOpModeRegistrar(final OpModeManager opModeManager) throws ClassNotFoundException, NoSuchMethodException, InvocationTargetException, IllegalAccessException, IOException
        {
        this.opModeManager             = opModeManager;
        this.classNameOverrides        = new HashMap<>();
        this.pairedOpModes             = new HashMap<>();
        this.classesSeen               = new HashSet<>();
        this.partialClassNamesToIgnore = new LinkedList<>();
        this.partialClassNamesToIgnore.add("com.google");
        this.partialClassNamesToIgnore.add("io.netty");

        // Find the file in which we are executing
        Class<?> activityThreadClass    = Class.forName("android.app.ActivityThread");
        Method methodCurrentApplication = activityThreadClass.getMethod("currentApplication");
        this.context                    = (Application) methodCurrentApplication.invoke(null, (Object[]) null);
        this.dexFile                    = new DexFile(context.getPackageCodePath());
        }

    //----------------------------------------------------------------------------------------------
    // Operations
    //----------------------------------------------------------------------------------------------

    private final String TAG = "AnnotatedOpModeReg";

    void doRegistration()
    // The body of this is from the following, without which we could not have been successful
    // in this endeavor.
    //      https://github.com/dmssargent/Xtensible-ftc_app/blob/master/FtcRobotController/src/main/java/com/qualcomm/ftcrobotcontroller/opmodes/FtcOpModeRegister.java
    // Many thanks.
        {
        // Find all the candidates
        this.findOpModesFromClassAnnotations();
        this.findOpModesFromRegistrarMethods();

        // Sort the linked lists within opModes
        Comparator<Class> comparator = new Comparator<Class>()
            {
            @Override public int compare(Class lhs, Class rhs)
                {
                if (lhs.isAnnotationPresent(TeleOp.class) && rhs.isAnnotationPresent(TeleOp.class))
                    return getOpModeName(lhs).compareTo(getOpModeName(rhs));

                else if (lhs.isAnnotationPresent(Autonomous.class) && rhs.isAnnotationPresent(TeleOp.class))
                    return 1;

                else if (lhs.isAnnotationPresent(TeleOp.class) && rhs.isAnnotationPresent(Autonomous.class))
                    return -1;

                else if (lhs.isAnnotationPresent(Autonomous.class) && rhs.isAnnotationPresent(Autonomous.class))
                    return getOpModeName(lhs).compareTo(getOpModeName(rhs));

                return -1;
                }
            };
        for (String key : pairedOpModes.keySet())
            {
            Collections.sort(pairedOpModes.get(key), comparator);
            }

        // "Sort the map by keys, after discarding the old keys, use the new key from
        // the first item in each LinkedList, and change from HashMap to TreeMap"
        TreeMap<String, LinkedList<Class>> sortedOpModes = new TreeMap<>();
        for (String key : pairedOpModes.keySet())
            {
            Class<? extends OpMode> opMode = pairedOpModes.get(key).getFirst();
            sortedOpModes.put(getOpModeName(opMode), pairedOpModes.get(key));
            }

        // Finally, register all the opmodes
        for (LinkedList<Class> opModeList : sortedOpModes.values())
            {
            for (Class opMode : opModeList)
                {
                this.opModeManager.register(getOpModeName(opMode), opMode);
                }
            }
        }

    /**
     * Find the list of OpMode classes which should be registered by looking
     * in the class annotations.
     */
    private void findOpModesFromClassAnnotations()
        {
        List<Class> allClasses = findAllClasses();

        for (Class clazz : allClasses)
            {
            // If the class doesn't extend OpMode, that's an error, we'll ignore it
            if (!isOpMode(clazz))
                continue;

            // If we have BOTH autonomous and teleop annotations on a class, that's an error, we'll ignore it.
            if (clazz.isAnnotationPresent(TeleOp.class) && clazz.isAnnotationPresent(Autonomous.class))
                continue;

            // If the class has been annotated as @Disabled, then ignore it
            if (clazz.isAnnotationPresent(Disabled.class))
                continue;

            // It passes all our tests, add it!
            addAnnotatedClass(clazz);
            }
        }

    private void findOpModesFromRegistrarMethods()
        {
        // This will, nicely, have duplicates removed. But it might contain methods
        // we can't actually invoke, so beware.
        Set<Method> methods = findOpModeRegistrarMethods();
        AnnotationOpModeManager manager = new AnnotationOpModeManager();
        for (Method method : methods)
            {
            try {
                method.invoke(null, manager);
                }
            catch (Exception e)
                {
                // ignored
                }
            }
        }

    /** Find the list of methods correctly tagged with @OpModeRegistrar */
    private Set<Method> findOpModeRegistrarMethods()
        {
        HashSet<Method> result = new HashSet<Method>();
        List<Class> allClasses = findAllClasses();
        for (Class clazz : allClasses)
            {
            List<Method> methods = Util.getDeclaredMethodsIncludingSuper(clazz);
            for (Method method : methods)
                {
                // Only look for those methods tagged as registrars
                if (!method.isAnnotationPresent(OpModeRegistrar.class))
                    continue;

                // Filter on method signature
                Class<?>[] parameters = method.getParameterTypes();
                if (parameters.length !=1)
                    continue;

                // We don't actually do any more formal parameter checking, as it's not
                // worth it. We'll just catch exceptions when we try to use the method if
                // it happens to have the wrong signture
                //
                // TODO: It would be nice if we did better error checking and could tell
                // the programmer what was going on.

                // Filter on modifiers.
                // TODO: allow non-public methods, but force their use?
                int requiredModifiers   = Modifier.STATIC | Modifier.PUBLIC;
                int prohibitedModifiers = Modifier.ABSTRACT;
                if (!((method.getModifiers() & requiredModifiers) == requiredModifiers && (method.getModifiers() & prohibitedModifiers) == 0))
                    continue;

                // Ok, it's a candidate
                result.add(method);
                }
            }
        return result;
        }

    class AnnotationOpModeManager implements IOpModeManager
        {
        public void register(Class opModeClass)
            {
            if (isOpMode(opModeClass)) // avoid downstream problems in our own code
                {
                addAnnotatedClass(opModeClass);
                }
            }

        public void register(String name, Class opModeClass)
            {
            if (isOpMode(opModeClass))
                {
                addClassWithName(opModeClass, name);
                }
            }

        public void register(String name, OpMode opModeInstance)
            {
            // We just go ahead and register this, as there's nothing else to do.
            // TODO: we could register these AFTER the classes, if we wanted to.
            opModeManager.register(name, opModeInstance);
            }
        }

    /**
     * Find all the classes in the context in which we should consider looking, which
     * currently?) is the entire .APK in which we are found.
     */
    private List<Class> findAllClasses()
        {
        // A list of annotated OpModes, grouped by their pairing properties (if any)
        List<Class> result = new LinkedList<Class>();

        // Iterate over all the classes in this whole .APK
        LinkedList<String> classNames = new LinkedList<>(Collections.list(dexFile.entries()));
        for (String className : classNames)
            {
            // Ignore classes that are in some domains we are to ignore
            // TODO: simple containment probably isn't the right test here
            boolean shouldIgnore = false;
            for (String domain : partialClassNamesToIgnore)
                {
                if (className.contains(domain))
                    {
                    shouldIgnore = true;
                    break;
                    }
                }
            if (shouldIgnore)
                continue;

            // Get the Class from the className
            Class clazz;
            try {
                clazz = Class.forName(className, false, context.getClassLoader());
                }
            catch (NoClassDefFoundError|ClassNotFoundException ex)
                {
                Log.w(TAG, className + " " + ex.toString(), ex);
                if (className.contains("$"))
                    {
                    className = className.substring(0, className.indexOf("$") - 1);
                    }
                partialClassNamesToIgnore.add(className);
                continue;
                }

            // Remember that class
            result.add(clazz);
            }

        return result;
        }

    /** add this class, which has annotations, to the map of classes to register */
    private void addAnnotatedClass(Class clazz)
        {
        // Locate TeleOp and Autonomous pairs
        if (clazz.isAnnotationPresent(TeleOp.class))
            {
            Annotation annotation = clazz.getAnnotation(TeleOp.class);
            String pairedName = ((TeleOp) annotation).pairWithAuto();
            addPairedClassWithName(clazz, pairedName);
            }

        if (clazz.isAnnotationPresent(Autonomous.class))
            {
            Annotation annotation = clazz.getAnnotation(Autonomous.class);
            String pairedName = ((Autonomous) annotation).pairWithTeleOp();
            addPairedClassWithName(clazz, pairedName);
            }
        }

    private void addPairedClassWithName(Class clazz, String pairedName)
        {
        if (pairedName.equals(""))
            addToOpModes(clazz);
        else
            addToOpModes(pairedName, clazz);
        }

    private void addClassWithName(Class clazz, String name)
        {
        addToOpModes(clazz);
        this.classNameOverrides.put(clazz, name);
        }

    /** Add a class to the map under a pairing key we make up, as the class is not paired */
    private void addToOpModes(Class clazz)
        {
        int i = 0;
        while (this.pairedOpModes.containsKey(Integer.toString(i)))
            i++;
        addToOpModes(Integer.toString(i), clazz);
        }

    /** Add a class to the map under the indicated key */
    private void addToOpModes(String pairingKey, Class clazz)
        {
        // Have we seen this class before?
        if (!this.classesSeen.contains(clazz))
            {
            this.classesSeen.add(clazz);

            if (this.pairedOpModes.containsKey(pairingKey))
                {
                this.pairedOpModes.get(pairingKey).add(clazz);
                }
            else
                {
                LinkedList<Class> temp = new LinkedList<>();
                temp.add(clazz);
                this.pairedOpModes.put(pairingKey, temp);
                }
            }
        }

    /** Returns the name we are to use for this class in the driver station display */
    private String getOpModeName(Class<? extends OpMode> opMode)
        {
        String name;

        if (this.classNameOverrides.containsKey(opMode))
            name = this.classNameOverrides.get(opMode);
        else if (opMode.isAnnotationPresent(TeleOp.class))
            name = opMode.getAnnotation(TeleOp.class).name();
        else if (opMode.isAnnotationPresent(Autonomous.class))
            name = opMode.getAnnotation(Autonomous.class).name();
        else
            name = opMode.getSimpleName();

        if (name.equals(""))
            name = opMode.getSimpleName();

        return name;
        }

    private boolean isOpMode(Class clazz)
        {
        return inheritsFrom(clazz, OpMode.class);
        }

    /** Answers whether one class is or inhertits from another */
    private static boolean inheritsFrom(Class baseClass, Class superClass)
        {
        while (baseClass != null)
            {
            if (baseClass == superClass)
                return true;
            baseClass = baseClass.getSuperclass();
            }
        return false;
        }


    }
