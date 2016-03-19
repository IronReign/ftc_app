package com.ironreignrobotics.resq;

/**
 * Created by Maximillian Virani on 3/19/2016.
 */

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class CsvLogKeeper{

    private static String variableName;
    private static FileWriter logKeeper;
    private static String fileName;
    private static boolean alreadyExists;
    private static File logFile;
    private static int dataLength;

    public CsvLogKeeper(String varname){

        dataLength = 1;
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

    public CsvLogKeeper(String title, int numvars, String varnames){

        dataLength = numvars;
        variableName = title;
        fileName = title+".csv";
        alreadyExists = new File(fileName).exists();
        logFile = new File(fileName);
        try{
            logKeeper = new FileWriter(logFile, true);
            if (!alreadyExists){
                logKeeper.append("Log of variables "+variableName);
                logKeeper.append("\n");
                logKeeper.append(varnames);
                logKeeper.append("\n");
                logKeeper.flush();
            }
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(ArrayList data){
        try{
            for(int i=0; i<dataLength; i++){
                logKeeper.append(data.get(i)+",");
            }
            logKeeper.append("\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(double data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(int data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(short data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(String data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(long data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(float data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(char data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(boolean data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(byte data){
        try{
            logKeeper.append(data+"\n");
            logKeeper.flush();
        }
        catch(IOException e){
            e.printStackTrace();
        }
    }

    public static void UpdateLog(Object data){
        try{
            logKeeper.append(data.toString()+"\n");
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
