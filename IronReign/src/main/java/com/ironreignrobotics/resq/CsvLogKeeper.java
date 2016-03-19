package com.ironreignrobotics.resq;

/**
 * Created by Maximillian Virani on 3/19/2016.
 */

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class CsvLogKeeper{

    private static String variableName;
    private static FileWriter logKeeper;
    private static String fileName;
    private static boolean alreadyExists;
    private static File logFile;

    public CsvLogKeeper(String varname){

        variableName = varname;
        fileName = varname+".csv";
        alreadyExists = new File(fileName).exists();
        logFile = new File(fileName);
        try{
            logKeeper = new FileWriter(logFile, true);
            if (!alreadyExists){
                logKeeper.append("Log of variable "+variableName);
                logKeeper.append("\n");
                logKeeper.flush();
            }
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(double data){
        try{
            logKeeper.append(data+",");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(int data){
        try{
            logKeeper.append(data+",");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(short data){
        try{
            logKeeper.append(data+",");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(String data){
        try{
            logKeeper.append(data+",");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(long data){
        try{
            logKeeper.append(data+",");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(float data){
        try{
            logKeeper.append(data+",");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(char data){
        try{
            logKeeper.append(data+",");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(boolean data){
        try{
            logKeeper.append(data+",");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(byte data){
        try{
            logKeeper.append(data+",");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(Object data){
        try{
            logKeeper.append(data.toString()+",");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void CloseLog(){
        try{
            logKeeper.append("\n");
            logKeeper.flush();
            logKeeper.close();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

}
