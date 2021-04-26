using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class TaskPauser
{
    CancellationTokenSource m_Source = new CancellationTokenSource();
    object m_Result;
    public object Result => m_Result;

    public async Task<object> PauseUntilResumed()
    {
        try
        {
            while (!m_Source.Token.IsCancellationRequested)
            {
                await Task.Delay(10000, m_Source.Token);
            }
        }
        catch(TaskCanceledException)
        {

        }
        return m_Result;
    }

    public void Resume(object result)
    {
        m_Result = result;
        m_Source.Cancel();
    }
}
