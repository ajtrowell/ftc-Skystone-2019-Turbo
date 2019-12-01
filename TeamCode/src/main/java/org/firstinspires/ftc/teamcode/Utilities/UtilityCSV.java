package org.firstinspires.ftc.teamcode.Utilities;

import android.util.Log;

import com.opencsv.CSVReader;
import com.opencsv.CSVWriter;

import org.openftc.revextensions2.ExpansionHubMotor;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.nio.Buffer;
import java.nio.file.Files;
import java.nio.file.Paths;


/**
 * UtilityCSV holds file path, CSVReader, and CSVWriter.
 */
public class UtilityCSV {

    private String TAG = "UtilityCSV";
    private String CSV_FILE_PATH;

    public CSVReader csvReader;
    public CSVWriter csvWriter;
//    private Reader reader;
//    private Writer writer;
    private boolean isReaderOpen = false;
    private boolean isWriterOpen = false;

    /**
     * Constructor
     * Sets up the file path.
     * @param CSV_FILE_PATH
     */
    public UtilityCSV(String CSV_FILE_PATH) {
        this.CSV_FILE_PATH = CSV_FILE_PATH;
    }

    /**
     * Create CSV Reader
     */
    public void createReader() {
        try {
            //Reader reader = Files.newBufferedReader(Paths.get(CSV_FILE_PATH));
            Reader reader = new BufferedReader( new FileReader(CSV_FILE_PATH));
            csvReader = new CSVReader(reader);
            isReaderOpen = true;
        } catch (Exception e) {
            throw(new RuntimeException("Could not create CSV reader"));
        }
    }

    /**
     * Create CSV Writer
     */
    public void createWriter() {
        try {
            Writer writer = new BufferedWriter( new FileWriter(CSV_FILE_PATH));
            csvWriter = new CSVWriter(writer);
            isWriterOpen = true;
        } catch (Exception e) {
            throw(new RuntimeException("Could not create CSV writer"));
        }
    }

    /**
     * Close both the reader and the writer.
     */
    public void close() {
        // Close Reader
        if(csvReader != null) {
            try {
                csvReader.close();
            } catch (IOException e) {
                e.printStackTrace();
                Log.e(TAG, e.getMessage());
            }
            isReaderOpen = false;
        }

        // Close Writer
        if(csvWriter != null) {
            try {
                csvWriter.close();
            } catch (IOException e) {
                e.printStackTrace();
                Log.e(TAG, e.getMessage());
            }
            isWriterOpen = false;
        }
    }
}
