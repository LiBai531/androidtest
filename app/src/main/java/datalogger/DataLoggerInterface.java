package datalogger;


public interface DataLoggerInterface
{
    void setHeaders(Iterable<String> headers) throws IllegalStateException;
    void addRow(Iterable<String> values) throws IllegalStateException;
    String writeToFile();
}
