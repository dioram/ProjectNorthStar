/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2018.                                 *
 * Leap Motion proprietary and confidential.                                  *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using NUnit.Framework;
using System;
using System.Threading;

namespace Leap.Unity.Tests
{

    public class ProduceConsumeBufferTest
    {

        private ProduceConsumeBuffer<TestStruct> buffer;

        [SetUp]
        public void Setup()
        {
            buffer = new ProduceConsumeBuffer<TestStruct>(16);
        }

        [TearDown]
        public void Teardown()
        {
            buffer = null;
        }

        [Test]
        [Timeout(1000)]
        public void Test()
        {
            Thread consumer = new Thread(new ThreadStart(consumerThread));
            Thread producer = new Thread(new ThreadStart(producerThread));

            consumer.Start();
            producer.Start();

            consumer.Join();
            producer.Join();
        }

        private void consumerThread()
        {
            try
            {
                for (int i = 0; i < buffer.Capacity; i++)
                {
                    TestStruct s;
                    s.index = i;
                    s.name = i.ToString();
                    while (!buffer.TryEnqueue(ref s)) { }
                }
            }
            catch (Exception e)
            {
                Assert.Fail(e.Message);
            }
        }

        private void producerThread()
        {
            try
            {
                for (int i = 0; i < buffer.Capacity; i++)
                {
                    TestStruct s;
                    while (!buffer.TryDequeue(out s)) { }

                    Assert.That(s.index, Is.EqualTo(i));
                    Assert.That(s.name, Is.EqualTo(i.ToString()));
                }
            }
            catch (Exception e)
            {
                Assert.Fail(e.Message);
            }
        }

        private struct TestStruct
        {
            public int index;
            public string name;
        }
    }
}
